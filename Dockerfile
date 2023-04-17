# syntax=docker/dockerfile:1 

FROM python:3.9-slim-buster 

WORKDIR /build 

# dependencies
COPY requirements.txt requirements.txt 
RUN pip3 install -r requirements.txt 

COPY . . 


# system configuration 
RUN apt-get update 
RUN apt-get install --yes build-essential
RUN pip3 install . 

CMD ["/bin/bash"] 
