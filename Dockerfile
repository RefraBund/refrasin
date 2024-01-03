FROM mcr.microsoft.com/dotnet/sdk:8.0-alpine AS build-stage

WORKDIR /refrasin
COPY . ./
RUN dotnet restore
RUN dotnet publish -c Release -o /build RefraSin

FROM mcr.microsoft.com/dotnet/sdk:8.0-alpine3.18

RUN echo "https://dl-cdn.alpinelinux.org/alpine/edge/testing" >> /etc/apk/repositories && apk update

RUN apk add \
    shadow \
    python3 \
    py3-matplotlib \
    py3-scipy \
    py3-pip \
    xonsh \
    gosu \
    musl-dev \
    gcc \
    linux-headers

RUN pip install \
    jupyterlab 
    
COPY ./docker /docker
    
ENV REFRASIN_UID=1000\
    REFRASIN_GID=1000\
    ORIG_REFRASIN_UID=$REFRASIN_UID\
    ORIG_REFRASIN_GID=$REFRASIN_GID

RUN addgroup -g $REFRASIN_GID refrasin && \
    adduser -D -u $REFRASIN_UID -G refrasin refrasin 

USER refrasin
ENV PATH="$PATH:/home/refrasin/.dotnet/tools"
RUN dotnet tool install -g Microsoft.dotnet-interactive && \
    dotnet interactive jupyter install

USER root
COPY --from=build-stage /build /refrasin
ENV APP_PATHS="/refrasin"
    
VOLUME "/userfiles"
WORKDIR  "/userfiles"
EXPOSE 8888

ENTRYPOINT ["/docker/entrypoint.xsh"]
CMD ["jupyter", "lab", "--ip=0.0.0.0"]