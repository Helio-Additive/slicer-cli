target "default" {
  context    = "."
  dockerfile = "./Dockerfile"
  tags       = ["slicer-cli:latest"]
}
