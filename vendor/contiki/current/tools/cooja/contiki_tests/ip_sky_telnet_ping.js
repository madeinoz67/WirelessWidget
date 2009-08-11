TIMEOUT(30000);

/* conf */
nrReplies = 0;
ipAddress = "172.16.1.0";
osName = java.lang.System.getProperty("os.name").toLowerCase();
if (osName.startsWith("win")) {
  pingOnceCmd = "ping -n 1 " + ipAddress;
  pingCmd = "ping -n 10 " + ipAddress;
} else {
  pingOnceCmd = "ping -c 1 " + ipAddress;
  pingCmd = "ping -c 10 " + ipAddress;
}
replyMsg = "from " + ipAddress;

/* wait for mote startup */
WAIT_UNTIL(msg.contains('Sky telnet process'));

/* make gateway */
pingOnceProcess  = new java.lang.Runtime.getRuntime().exec(pingOnceCmd);
GENERATE_MSG(5000, "continue");
WAIT_UNTIL(msg.equals("continue"));
log.log("cont\n");

/* override simulation delay, test will time out is too fast otherwise */
mote.getSimulation().setDelayTime(1);

/* start ping process */
var runnableObj = new Object();
runnableObj.run = function() {
  pingProcess  = new java.lang.Runtime.getRuntime().exec(pingCmd);
  log.log("cmd> " + pingCmd + "\n");

  stdIn = new java.io.BufferedReader(new java.io.InputStreamReader(pingProcess.getInputStream()));
  while ((line = stdIn.readLine()) != null) {
    log.log("> " + line + "\n");
    if (line.contains(replyMsg)) {
      nrReplies++;
      //log.log("reply #" + nrReplies + "\n");
    }
  }
  pingProcess.destroy();

  if (nrReplies > 5) {
    log.testOK(); /* Report test success and quit */
  } else {
    log.log("Only " + nrReplies + "/10 ping replies was received\n");
    log.testFailed();
  }
}
var thread = new java.lang.Thread(new java.lang.Runnable(runnableObj));
thread.start();
