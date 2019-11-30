from spear-env

COPY . /software

RUN apt-get update
RUN ( cd software && bash ./unpack.sh dev )
