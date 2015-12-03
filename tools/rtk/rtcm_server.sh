#!/bin/bash

SERVER=../../../rtklib/app/str2str/gcc/str2str 

$SERVER -in ntrip://ntlab:ntlab1234@212.98.190.90:8080/BelarusVRS -out tcpsvr://:40007 -n 1000 -p 53.9665 27.5908 250
