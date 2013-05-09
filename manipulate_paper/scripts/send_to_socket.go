package main

import (
	"net"
	"os"
	"fmt"
	"bufio"
)

func main() {
	args := os.Args;
	if len(args)!=3{
		println("You must provide two arguments")
		return
	}
	ln, err := net.Dial("tcp",":" + args[1]);
	if err != nil {
		println("Dial error: ", err.Error())
		return
	}
	fmt.Fprintf(ln,args[2]);
	status, err := bufio.NewReader(ln).ReadString('\n')
	println(status);
	ln.Close()
	return
}
