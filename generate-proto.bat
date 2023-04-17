@echo off

if [%2]==[] (
	echo Usage: generate-proto SketchDir filename
	echo Generates [SketchDir]/src/[filename].pb.h 
	exit /b 1
)

nanopb_generator -I Protobuf -D %1/src -L "#include \"utils/src/%%s\"" %2.proto