# Security Policy

## Reporting a vulnerability

**Do not open a public GitHub issue for security vulnerabilities.**

Email: security@helioadditive.com

Include:
- Description of the vulnerability
- Reproduction steps
- Affected versions / platforms
- Any known mitigations

You will receive an acknowledgement within 2 business days. We aim to ship a
patch within 30 days of confirmation for high/critical severity issues.

## Disclosure policy

We follow coordinated disclosure. Once a patch is shipped we will publish a
security advisory with full details, credit to the reporter, and the affected
version range.

## Scope

`slicer-cli` is a local command-line binary. The primary attack surface is:

1. **Malicious 3MF / STL input** — the parser processes potentially-untrusted
   geometry. Memory-safety bugs in the C++ parsing path are in scope.
2. **Profile injection** — malicious process / printer / filament profile JSON
   that causes unintended gcode output. In scope.
3. **Subprocess boundary** — `slicer-cli` is subprocess-invoked by the Helio
   closed apps; command-injection through filenames or profile values is in
   scope.

**Out of scope:**
- The excluded feature set (SLA hollowing, mesh cutting, post-processor scripts)
  — these features are deliberately not compiled in; issues in excluded code
  are noted but not treated as production vulnerabilities until the feature
  is re-enabled.
- Attacks that require physical access to the machine running the binary.
- Social engineering.

## Lapsed-user security exception

Critical CVE patches in `slicer-cli` will be released and delivered to all
users regardless of subscription status — the subscription covers feature
updates, not security patches. The Helio closed apps' update policy documents
this separately.
