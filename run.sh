git pull origin dev
docker-compose down
docker image prune -f
docker-compose up --build 