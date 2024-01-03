FROM archlinux:latest

RUN pacman -Syu --noconfirm \
    dotnet-sdk \
    aspnet-runtime \
    nuget \
    jupyterlab \
    python-matplotlib \
    python-scipy \
    python-hatch \
    python-pipenv \
    python-pipx \
    skia-sharp \
    zsh \
    xonsh

ENV REFRASIN_UID=1000\
    REFRASIN_GID=1000\
    ORIG_REFRASIN_UID=$REFRASIN_UID\
    ORIG_REFRASIN_GID=$REFRASIN_GID

RUN groupadd -g $REFRASIN_GID refrasin && \
    useradd -u $REFRASIN_UID -g $REFRASIN_GID -s /usr/bin/zsh -m refrasin

COPY . /refrasin

VOLUME "/userfiles"

EXPOSE 8888
WORKDIR  "/userfiles"
ENTRYPOINT ["/refrasin/docker/docker-entrypoint.xsh"]
CMD ["--ip=0.0.0.0"]