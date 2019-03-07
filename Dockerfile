from spear-env

COPY . /rover2

RUN ( cd rover2 && bash ./unpack.sh dev )
