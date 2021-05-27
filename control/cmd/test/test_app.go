package main

import (
	"bufio"
	"log"
	"net"
	"os"
	"os/exec"
	"strings"
	"time"
)

func keyListener(pressCh chan []byte) {
	b := []byte{}
	for {
		os.Stdin.Read(b)

		pressCh <- b
	}
}

func handleConnection(c net.Conn, pressCh chan []byte) {
	log.Printf("Connection! from %s to %s\n", c.RemoteAddr(), c.LocalAddr())
	goo := true
	strCh := make(chan string, 100)


	timer := time.NewTimer(time.Second * 60)

	go func(strCh chan string) {
		for {
			netData, _ := bufio.NewReader(c).ReadString('\n')
			strCh <- netData
		}
	}(strCh)

	for ;goo; {
		select {
		case <-timer.C:
			goo = false
			log.Println("Expired")
		case netData := <- strCh:
			if !strings.Contains(netData, "PING") {
				log.Print("Data-> ", netData)
			}
			data := strings.TrimSpace(netData)
			timer.Reset(time.Millisecond * 5000)
			if data == "" {
				goo = false
			}

		}
	}
	c.Close()
}

// "server" for platform debugging
func main() {
	pressCh := make(chan []byte)
	go keyListener(pressCh)
	exec.Command("stty", "-F", "/dev/tty", "cbreak", "min", "1").Run()
	addrs := []string{"localhost:10002", "localhost:10001"}
	lns := []net.Listener{}
	for _, addr := range addrs {
		ln, _ := net.Listen("tcp", addr)
		lns = append(lns, ln)
	}
	for _, ln := range lns {
		go func(ln net.Listener) {
			for {
				log.Println("Listening", ln.Addr())
				conn, _ := ln.Accept()
				go handleConnection(conn, pressCh)
			}
		}(ln)
	}
	select {}
}
