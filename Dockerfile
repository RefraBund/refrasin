FROM mcr.microsoft.com/dotnet/sdk:8.0-alpine AS build-stage

WORKDIR /refrasin
COPY . ./
RUN dotnet restore
RUN dotnet publish -c Release -o /build RefraSin

FROM mcr.microsoft.com/dotnet/runtime:8.0-alpine

COPY --from=build-stage /build /refrasin
ENV APP_PATHS="/refrasin"

RUN echo "https://dl-cdn.alpinelinux.org/alpine/edge/testing" >> /etc/apk/repositories && apk update

RUN apk add \
    shadow \
    python3 \
    py3-matplotlib \
    py3-scipy \
    jupyter-notebook \
    xonsh \
    gosu

COPY ./docker /docker
WORKDIR /docker
    
ENV REFRASIN_UID=1000\
    REFRASIN_GID=1000\
    ORIG_REFRASIN_UID=$REFRASIN_UID\
    ORIG_REFRASIN_GID=$REFRASIN_GID

RUN addgroup -g $REFRASIN_GID refrasin && \
    cat /etc/passwd && \
    adduser -D -u $REFRASIN_UID -G refrasin refrasin
    
VOLUME "/userfiles"
WORKDIR  "/userfiles"
EXPOSE 8888

ENTRYPOINT ["/docker/entrypoint.xsh"]
CMD ["jupyter", "notebook", "--ip=0.0.0.0"]