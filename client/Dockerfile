FROM ubuntu:20.04
WORKDIR /interop/client

# Set the time zone to the competition time zone.
RUN ln -sf /usr/share/zoneinfo/America/New_York /etc/localtime

RUN apt-get -qq update && apt-get -qq install -y \
        libxml2-dev \
        libxslt-dev \
        protobuf-compiler \
        python2 \
        python3 \
        python3-dev \
        python3-venv \
        python3-nose \
        python3-pip \
        python3-pyproj \
        python3-lxml \
        sudo

RUN bash -c "cd /interop/client && \
            add-apt-repository universe && \
            curl https://bootstrap.pypa.io/get-pip.py --output get-pip.py && \
            python2 get-pip.py && \
            python2 -m pip install virtualenv && \
            source env/bin/activate && \
            pip install -r requirements.txt && \
            deactivate"

COPY client/requirements.txt .

RUN bash -c "cd /interop/client && \
        python3 -m venv --system-site-packages venv && \
        source venv/bin/activate && \
        pip3 install -r requirements.txt && \
        deactivate"

COPY proto/ ../proto
COPY client/ .
RUN bash -c "cd /interop/client && \
        source venv/bin/activate && \
        python3 setup.py install && \
        deactivate"

CMD bash --init-file configure.sh
