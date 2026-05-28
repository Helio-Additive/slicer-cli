#!/usr/bin/env bun
import { $ } from "bun";
import { Octokit } from "@octokit/rest";

const upstreamOwner = env("BAMBU_UPSTREAM_OWNER", "bambulab");
const upstreamRepo = env("BAMBU_UPSTREAM_REPO", "BambuStudio");
const submodulePath = env(
  "BAMBU_SUBMODULE_PATH",
  "libslic3r/bambustudio/references/BambuStudio",
);
const githubRepository = mustEnv("GITHUB_REPOSITORY");
const token = mustEnv("GH_TOKEN", "GITHUB_TOKEN");
const baseBranch = env("BAMBU_UPGRADE_BASE_BRANCH", env("GITHUB_REF_NAME", "main"));

const [repoOwner, repoName] = parseRepository(githubRepository);
const octokit = new Octokit({ auth: token });

const latestRelease = await latestStableRelease();
const latestSha = await releaseTargetSha(latestRelease.tag_name);
const currentSha = await currentSubmoduleSha(submodulePath);

if (currentSha === latestSha) {
  console.log(
    `BambuStudio submodule is already up to date at stable release ${latestRelease.tag_name} (${shortSha(currentSha)}).`,
  );
  process.exit(0);
}

const branchName = `bump/bambustudio-${safeRefPart(latestRelease.tag_name)}`;
const title = `Bump BambuStudio submodule to ${latestRelease.tag_name}`;

const existingPr = await findOpenPullRequest(branchName);
if (existingPr) {
  console.log(`Upgrade PR already exists: ${existingPr.html_url}`);
  await ensureIssue(title, currentSha, latestSha, existingPr.html_url);
  process.exit(0);
}

await configureGit();
await $`git fetch origin ${baseBranch} --depth=1`;
await $`git checkout -B ${branchName} FETCH_HEAD`;
await $`git submodule update --init --recursive -- ${submodulePath}`;
await $`git -C ${submodulePath} fetch origin tag ${latestRelease.tag_name} --depth=1`;
await $`git -C ${submodulePath} checkout ${latestSha}`;
await $`git add ${submodulePath}`;

if (!(await hasStagedChanges())) {
  console.log("No staged changes after updating the submodule.");
  process.exit(0);
}

await $`git commit -m ${title}`;
await $`git push --force-with-lease --set-upstream origin ${branchName}`;

const issue = await ensureIssue(title, currentSha, latestSha);
const pr = await octokit.pulls.create({
  owner: repoOwner,
  repo: repoName,
  title,
  head: branchName,
  base: baseBranch,
  body: [
    `Updates \`${submodulePath}\` to the latest stable BambuStudio release.`,
    "",
    `- Release: ${latestRelease.html_url}`,
    `- Tag: \`${latestRelease.tag_name}\``,
    `- Previous: ${commitUrl(currentSha)}`,
    `- Latest: ${commitUrl(latestSha)}`,
    issue ? `- Tracking issue: #${issue.number}` : undefined,
    "",
    "Opening this PR should trigger the normal build and test workflows.",
  ]
    .filter(Boolean)
    .join("\n"),
});

if (issue) {
  await octokit.issues.createComment({
    owner: repoOwner,
    repo: repoName,
    issue_number: issue.number,
    body: `Opened upgrade PR: ${pr.data.html_url}`,
  });
}

console.log(`Created upgrade PR: ${pr.data.html_url}`);

function env(name: string, fallback: string): string {
  return process.env[name] || fallback;
}

function mustEnv(...names: string[]): string {
  for (const name of names) {
    const value = process.env[name];
    if (value) {
      return value;
    }
  }
  throw new Error(`missing required environment variable: ${names.join(" or ")}`);
}

function parseRepository(repository: string): [string, string] {
  const parts = repository.split("/");
  if (parts.length !== 2 || !parts[0] || !parts[1]) {
    throw new Error(`GITHUB_REPOSITORY must be owner/repo, got: ${repository}`);
  }
  return [parts[0], parts[1]];
}

async function currentSubmoduleSha(path: string): Promise<string> {
  const output = await quiet($`git ls-tree HEAD -- ${path}`);
  const match = output.stdout.toString().match(/\bcommit\s+([0-9a-f]{40})\b/);
  if (!match) {
    throw new Error(`could not read submodule SHA from git ls-tree for ${path}`);
  }
  return match[1];
}

async function latestStableRelease() {
  for await (const response of octokit.paginate.iterator(octokit.repos.listReleases, {
    owner: upstreamOwner,
    repo: upstreamRepo,
    per_page: 100,
  })) {
    const release = response.data.find((candidate) => {
      return !candidate.draft && !candidate.prerelease;
    });
    if (release) {
      return release;
    }
  }
  throw new Error(`no stable releases found for ${upstreamOwner}/${upstreamRepo}`);
}

async function releaseTargetSha(tag: string): Promise<string> {
  const ref = await octokit.git.getRef({
    owner: upstreamOwner,
    repo: upstreamRepo,
    ref: `tags/${tag}`,
  });

  if (ref.data.object.type === "commit") {
    return ref.data.object.sha;
  }

  if (ref.data.object.type !== "tag") {
    throw new Error(`release tag ${tag} points to unsupported object type ${ref.data.object.type}`);
  }

  const tagObject = await octokit.git.getTag({
    owner: upstreamOwner,
    repo: upstreamRepo,
    tag_sha: ref.data.object.sha,
  });

  if (tagObject.data.object.type !== "commit") {
    throw new Error(
      `annotated release tag ${tag} points to unsupported object type ${tagObject.data.object.type}`,
    );
  }

  return tagObject.data.object.sha;
}

async function configureGit(): Promise<void> {
  await $`git config user.name ${env("BAMBU_UPGRADE_GIT_NAME", "bambustudio-upgrade-bot")}`;
  await $`git config user.email ${env(
    "BAMBU_UPGRADE_GIT_EMAIL",
    "bambustudio-upgrade-bot@users.noreply.github.com",
  )}`;
  await $`git remote set-url origin ${authenticatedRemoteUrl()}`;
}

async function hasStagedChanges(): Promise<boolean> {
  const output = await quiet($`git diff --cached --name-only`);
  return output.stdout.toString().trim().length > 0;
}

async function findOpenPullRequest(branchName: string) {
  const prs = await octokit.pulls.list({
    owner: repoOwner,
    repo: repoName,
    state: "open",
    head: `${repoOwner}:${branchName}`,
    per_page: 1,
  });
  return prs.data[0];
}

async function ensureIssue(
  title: string,
  previousSha: string,
  latestSha: string,
  prUrl?: string,
) {
  const issues = await octokit.search.issuesAndPullRequests({
    q: `repo:${repoOwner}/${repoName} is:issue is:open in:title "${title}"`,
    per_page: 1,
  });
  const existing = issues.data.items[0];
  if (existing) {
    return { number: existing.number, html_url: existing.html_url };
  }

  const issue = await octokit.issues.create({
    owner: repoOwner,
    repo: repoName,
    title,
    body: [
      "A new stable BambuStudio release is available for the pinned submodule.",
      "",
      `- Release: ${latestRelease.html_url}`,
      `- Tag: \`${latestRelease.tag_name}\``,
      `- Previous: ${commitUrl(previousSha)}`,
      `- Latest: ${commitUrl(latestSha)}`,
      prUrl ? `- Existing PR: ${prUrl}` : undefined,
    ]
      .filter(Boolean)
      .join("\n"),
  });
  return { number: issue.data.number, html_url: issue.data.html_url };
}

function commitUrl(sha: string): string {
  return `https://github.com/${upstreamOwner}/${upstreamRepo}/commit/${sha}`;
}

function authenticatedRemoteUrl(): string {
  return `https://x-access-token:${encodeURIComponent(token)}@github.com/${repoOwner}/${repoName}.git`;
}

function shortSha(sha: string): string {
  return sha.slice(0, 12);
}

function safeRefPart(value: string): string {
  return value.replace(/[^A-Za-z0-9._-]+/g, "-").replace(/^-+|-+$/g, "");
}

async function quiet(command: any): Promise<{ stdout: Buffer }> {
  return command.quiet();
}
