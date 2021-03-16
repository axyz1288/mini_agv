# Mini-AGV
Mini-AGV is a **low cost** and **small volume** AGV with **AI algorithm**. With Mini-AGV, you can reduce your human resource and increase productivity.
# Spec
* Embedding system: **nvidia nano**
* Motor: **robotis mx-106** x 4
* Lidar: **Slamtec M1M1**
* Battery: **ELLMAX 5800mAh** x 2   
# Ability
* Max velocity: **0.3 m/s**
* Max Load: **30kgw**
# Setup Process
## Master   
1. Install Microk8s and kubelet CRI change to docker.
2. Set Docker default runtime to nvidia docker runtime.
3. Lable master node agent=master and all agv nodes agent=agv
4. Set apiserver flag --allow-privileged=true
5. Deploy configmaps, storage/master and master.yaml.
## AGV
1. Set Jetson nano username to agv for mount path and desktop name to agv1, agv2, ... agvN for ros.
2. Disable nano power_save (sudo iw dev wlan0 set power_save off).
3. Disable swap permanently.(sudo swapoff -a && sudo sed -i.bak '/ swap / s/^\(.*\)$/#\1/g' /etc/fstab)
3. Install Microk8s and disable ha-cluster
4. Deployment storage/agv and agv.yaml.
# Notice
1. tf 
2. map.stcm path (Dockerfile & AGV.cpp)
# Support Information
Email: axyz1350@gmail.com

---   
Copyright © Eric_Chang. All rights reserved