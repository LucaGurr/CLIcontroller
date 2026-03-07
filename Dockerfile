FROM alpine:3.19
RUN apk add --no-cache gcc musl-dev bash
WORKDIR /app
COPY control.c .
COPY entrypoint.sh .
# Override GPIO_BASE_PATH to match the simulated sysfs volume at /gpio
RUN gcc -o control control.c -lm -DGPIO_BASE_PATH='"/gpio"'
RUN chmod +x entrypoint.sh
ENTRYPOINT ["/app/entrypoint.sh"]
