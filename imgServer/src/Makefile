CC =g++
#CC = arm-linux-gnueabihf-g++
#CFLAGS = -std=c++11 
CFLAGS = -pthread 
CFLAGS += -O3 -mcpu=cortex-a8 -mfpu=neon -mfloat-abi=hard -ftree-vectorize -ftree-vectorizer-verbose=3
OUTPUTNAME = FlyCamServer${D} 
INCLUDE =-I../include -I/../lib -I/usr/arm-linux-gnueabihf/lib  
#Original de Olivier, mais marche pas sur ubuntu.
#LIBS = -L../lib -lflycapture${D} -lprotobuf -lpthread
LIBS =-L/usr/lib -lflycapture${D} -lprotobuf -lpthread

OUTDIR = ..
PROTODIR = proto

PROTOBIN = protoc 
#PROTOBIN = ../bin/protoc 

OBJS = main.o LedsFinder.o Camera.o lodepng.o lz4.o
PROTOOBJS =${PROTODIR}/img.pb.o ${PROTODIR}/command.pb.o

#===src===-lprotobuf
D: ${PROTOOBJS} ${OBJS} 
	${CC} -o ${OUTPUTNAME} ${OBJS} ${PROTOOBJS} ${LIBS} -lpthread ${COMMON_LIBS} 
	mv ${OUTPUTNAME} ${OUTDIR}

%.o: %.cpp
	${CC} ${CFLAGS} ${INCLUDE}-c $*.cpp -lpthread

#===Proto===
${PROTODIR}/%.pb.h ${PROTODIR}/%.pb.cc: ../${PROTODIR}/%.proto
	${PROTOBIN} -I=../proto/ --cpp_out=${PROTODIR}/ ../proto/$*.proto
#-lprotobuf
${PROTODIR}/%.pb.o: ${PROTODIR}/%.pb.cc
	${CC} ${CFLAGS} ${INCLUDE} -c $< -o $@  

#===Clean===
clean_obj:
	rm -f ${OBJS} ${PROTOOBJS}	
	@echo "all cleaned up!"

clean:
	rm -f ${OUTDIR}/${OUTPUTNAME} ${OBJS} ${PROTOOBJS}
	@echo "all cleaned up!"
