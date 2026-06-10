variable "IMAGE_NAME" {
  default = "slicer-cli"
}

variable "IMAGE_TAG" {
  default = "latest"
}

target "slicer-cli" {
  context    = "."
  dockerfile = "Dockerfile"
  tags       = ["${IMAGE_NAME}:${IMAGE_TAG}"]
}
