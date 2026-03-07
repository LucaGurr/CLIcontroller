FROM alpine:3.19
RUN apk add --no-cache gcc musl-dev bash
WORKDIR /app
COPY control.c .
COPY entrypoint.sh .
RUN gcc -o control control.c -lm
RUN chmod +x entrypoint.sh
ENTRYPOINT ["/app/entrypoint.sh"]
