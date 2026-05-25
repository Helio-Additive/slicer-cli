variable "IMAGE_NAME" {
  default = "slicer-cli"
}

variable "IMAGE_TAG" {
  default = "local"
}

group "default" {
  targets = ["slicer-cli"]
}

target "slicer-cli" {
  context    = "."
  dockerfile = "Dockerfile"
  target     = "runtime"
  tags       = ["${IMAGE_NAME}:${IMAGE_TAG}"]
}
