# frontier-fly, headless: the search-and-rescue mission, FrontierBench and plan export in a container.
#
#   docker run --rm ghcr.io/alexnieves-cs/frontier-fly mission --scenario cabin-day
#   docker run --rm -v "$PWD:/work" ghcr.io/alexnieves-cs/frontier-fly bench --pilot guarded --out result.json
#
# The image holds one bundled ES module (the TypeScript sim, built by `npm run build:cli`), the
# shipped policy and the floor-plan set. No browser, GPU or npm packages at run time.

# Override to build from a registry mirror, e.g. --build-arg NODE_IMAGE=mirror.gcr.io/library/node:22-bookworm-slim
ARG NODE_IMAGE=node:22-bookworm-slim

# The bundle is plain JavaScript, so it is built once on the build machine for every platform.
FROM --platform=$BUILDPLATFORM ${NODE_IMAGE} AS build
WORKDIR /src
COPY package.json package-lock.json ./
RUN npm ci --no-audit --no-fund --ignore-scripts
COPY tsconfig.json LICENSE ./
COPY src ./src
COPY scripts ./scripts
COPY public/checkpoints/policy.bin ./public/checkpoints/policy.bin
COPY public/plans ./public/plans
RUN npm run build:cli && node dist-cli/frontier-fly.mjs list > /dev/null && mkdir /work

FROM ${NODE_IMAGE}
ARG VERSION=dev
ARG REVISION=unknown
LABEL org.opencontainers.image.title="frontier-fly" \
      org.opencontainers.image.description="Headless frontier-fly: a search-and-rescue drone autopilot sim, FrontierBench scorer and QGroundControl plan export" \
      org.opencontainers.image.source="https://github.com/alexnieves-cs/frontier-fly" \
      org.opencontainers.image.url="https://alexnieves-cs.github.io/frontier-fly/" \
      org.opencontainers.image.documentation="https://github.com/alexnieves-cs/frontier-fly/blob/main/docs/CLI.md" \
      org.opencontainers.image.licenses="MIT" \
      org.opencontainers.image.version="${VERSION}" \
      org.opencontainers.image.revision="${REVISION}"
COPY --from=build /src/dist-cli /opt/frontier-fly
# Output files land in the working directory; mount one at /work to keep them. (No RUN here, so
# the arm64 image needs no emulation to build.)
COPY --from=build --chown=node:node /work /work
WORKDIR /work
USER node
ENTRYPOINT ["node", "/opt/frontier-fly/frontier-fly.mjs"]
CMD ["--help"]
