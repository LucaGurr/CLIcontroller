FROM alpine:3.19
RUN apk add --no-cache bash coreutils
COPY setup.sh monitor.sh /
RUN chmod +x /setup.sh /monitor.sh
ENTRYPOINT ["/bin/bash", "/setup.sh"]
