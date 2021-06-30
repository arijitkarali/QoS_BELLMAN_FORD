set val(chan)         Channel/WirelessChannel  ;# Channel type
set val(prop)         Propagation/TwoRayGround ;# Radio-propagation model
set val(ant)          Antenna/OmniAntenna      ;# Antenna type
set val(ll)           LL                       ;# Link layer type
set val(ifq)          Queue/DropTail/PriQueue  ;# Interface queue type
set val(ifqlen)       50                       ;# Max packet in ifq
set val(netif)        Phy/WirelessPhy          ;# Network interface type
set val(mac)          Mac/802_11               ;# MAC type
set val(rp)           AODV                     ;# Routing protocol
set val(x)            800		       ;# X length
set val(y)            800		       ;# Y length
set val(finish)       10		       ;# Finish time
set val(nn)           10		       ;# Number of mobilenodes

set ns_ [new Simulator]

set trf [open out.tr w]
$ns_ trace-all $trf 

set namtrace [open out.nam w]
$ns_ namtrace-all-wireless $namtrace $val(x) $val(y)

set topo [new Topography]
$topo load_flatgrid $val(x) $val(y)
 
set god_ [create-god $val(nn)]
set chan_1 [new $val(chan)]

$ns_ node-config  -adhocRouting $val(rp) \
          -llType $val(ll) \
                 -macType $val(mac) \
                 -ifqType $val(ifq) \
                 -ifqLen $val(ifqlen) \
                 -antType $val(ant) \
                 -propType $val(prop) \
                 -phyType $val(netif) \
                 -topoInstance $topo \
                 -agentTrace ON \
                 -routerTrace ON \
                 -macTrace ON \
                 -movementTrace ON \
                 -channel $chan_1

for {set i 0} {$i < $val(nn) } { incr i } {
	set node_($i) [$ns_ node]
	$ns_ initial_node_pos $node_($i) 35
}

source mob
source sessions

$ns_ at 0.0 "$node_(1) color blue"
$node_(1) color "blue"
$ns_ at 0.0 "$node_(2) color orange"
$node_(2) color "blue"



proc finish {} {
    global ns_ namtrace filename
    $ns_ flush-trace
    close $namtrace  
    exec nam out.nam &
    exit 0
}

$ns_ at $val(finish) "finish"
puts "Start of simulation..."
$ns_ run
