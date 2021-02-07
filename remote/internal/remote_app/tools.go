package remote_app

import "log"

func FailedOnError(err error, msg string) {
	if err != nil {
		log.Printf("Error: %s\nMessage: %s\n", err, msg)
	}
}
