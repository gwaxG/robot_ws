package ros

n, err := goroslib.NewNode(goroslib.NodeConf{
Name:          "goroslib_sp",
MasterAddress: "127.0.0.1:11311",
})
if err != nil {
panic(err)
}
defer n.Close()

// create a service provider
sp, err := goroslib.NewServiceProvider(goroslib.ServiceProviderConf{
Node:     n,
Name:     "test_srv",
Srv:      &Service{},
Callback: onService,
})
if err != nil {
panic(err)
}
defer sp.Close()

// freeze main loop
select {}

