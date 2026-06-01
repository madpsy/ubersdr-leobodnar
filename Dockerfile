# Build stage
FROM gcc:14-bookworm AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
        libudev-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build

COPY lbe-142x.c hidapi.h ./

RUN gcc -O2 -o lbe-142x lbe-142x.c -I. -lpthread

# Runtime stage
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
        libudev1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY --from=builder /build/lbe-142x /usr/local/bin/lbe-142x

EXPOSE 5123

ENTRYPOINT ["lbe-142x"]
CMD ["--serve"]
