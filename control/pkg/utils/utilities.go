package utils

import (
	"flag"
	"log"
)

func Flags() (bool, bool){
	isPlatform := flag.Bool("real", false, "Is the platform used?")
	isRos := flag.Bool("ros", false, "Is ROS used in reality?")
	flag.Parse()
	return *isPlatform, *isRos
}

func FailOnError(err error, msg string) {
	if err != nil {
		log.Panicln(msg)
	}
}