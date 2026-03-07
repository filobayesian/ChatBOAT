#!/bin/bash
# Build the ChatBOAT Docker image and push to Docker Hub.
# Usage: ./scripts/build_and_push.sh <dockerhub_username>
#   e.g. ./scripts/build_and_push.sh myuser
# Then use "myuser/chatboat-sim:latest" as the RunPod template image.
set -euo pipefail

USERNAME="${1:?Usage: $0 <dockerhub_username>}"
IMAGE="${USERNAME}/chatboat-sim:latest"

echo "=== Building ChatBOAT image ==="
docker build -t "$IMAGE" -f docker/Dockerfile .

echo "=== Pushing to Docker Hub ==="
docker push "$IMAGE"

echo ""
echo "=== Done! ==="
echo "Image: $IMAGE"
echo ""
echo "Next steps — create a RunPod Pod:"
echo "  1. Go to https://www.runpod.io/console/pods"
echo "  2. Click '+ GPU Pod'"
echo "  3. Select RTX A4000 (cheapest with OpenGL 4.6)"
echo "  4. Under 'Container Image', enter: $IMAGE"
echo "  5. Under 'Expose HTTP Ports', add: 6080"
echo "  6. Set Container Disk to 20 GB, Volume Disk to 0 GB"
echo "  7. Click 'Deploy On-Demand'"
echo "  8. Once running, click 'Connect' -> port 6080 to open noVNC"
