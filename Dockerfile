from spear-env

COPY . /software

RUN ( cd software && bash ./unpack.sh dev )
