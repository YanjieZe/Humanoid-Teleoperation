
# Installation for Teleoperation on Humanoid Robot

**Install realsense driver for L515**. https://github.com/IntelRealSense/librealsense/releases/tag/v2.54.2


**Teleop with Vision Pro.** to use vision pro teleop, first, install our app on vision pro. then install the following packages:
    
    # I assume you have not installed iDP3 env. You could install the teleop code first, and then install iDP3.
    conda remove -n idp3 --all
    conda create -n idp3 python=3.11
    conda activate idp3

    pip install avp_stream sapien==3.0.b0 tyro anytree trimesh pytransform3d nlopt
    pip install cmeel-urdfdom==3.1.1.1 pin==2.7.0
    pip install pyrealsense2==2.54.2.5684
    pip install quaternionic tqdm 
    pip install vuer==0.0.32rc7

    cd  teleop-zenoh
    cd libs
    pip install robot_rcs-0.4.0.7-cp311-cp311-manylinux_2_30_x86_64.whl
    pip install robot_rcs_gr-1.9.1.4-cp311-cp311-manylinux_2_30_x86_64.whl

    sudo apt-get update -y
    # libtinyxml.so.2.6.2
    sudo apt install libtinyxml2.6.2v5
    #  libtinyxml2.so.6
    sudo apt-get install -y libtinyxml2-6
    sudo apt install libconsole-bridge0.4

    pip install rerun-sdk termcolor h5py flask plotly


# Setup Local Vision Stream

Apple does not allow WebXR on non-https connections. To test the application locally, we need to create a self-signed certificate and install it on the client. You need a ubuntu machine and a router. Connect the VisionPro and the ubuntu machine to the same router. 

1. install mkcert: https://github.com/FiloSottile/mkcert
```
sudo apt install libnss3-tools
curl -JLO "https://dl.filippo.io/mkcert/latest?for=linux/amd64"
chmod +x mkcert-v*-linux-amd64
sudo cp mkcert-v*-linux-amd64 /usr/local/bin/mkcert
```
2. check local ip address: 

```
ifconfig | grep inet
```

Suppose the local ip address of the ubuntu machine is `192.168.31.157`.

3. create certificate: 

```
mkcert -install && mkcert -cert-file cert.pem -key-file key.pem 192.168.31.157 localhost 127.0.0.1
```

4. open firewall on server

```
sudo iptables -A INPUT -p tcp --dport 8012 -j ACCEPT
sudo iptables-save
sudo iptables -L
```

or can be done with `ufw`:

```
sudo ufw allow 8012
```

5.

```python
self.app = Vuer(host='0.0.0.0', cert="./cert.pem", key="./key.pem")
```

6. install ca-certificates on VisionPro

```
mkcert -CAROOT
```

Copy the rootCA.pem via AirDrop to VisionPro and install it.

Settings > General > About > Certificate Trust Settings. Under "Enable full trust for root certificates", turn on trust for the certificate.

settings > Apps > Safari > Advanced > Feature Flags > Enable WebXR Related Features

7. open the browser on Safari on VisionPro and go to `https://192.168.31.157:8012?ws=wss://192.168.31.157:8012`

8. Click `Enter VR` and ``Allow`` to start the VR session.


# Connect Vision Pro and Your Mac for develop

What steps did you take to get the device working in Xcode. The following steps should get you started:

Vision Pro

    Navigate to Settings > General > Remote Devices and remove any computer listed.

Xcode

    Navigate to Window > Devices and Simulators.
    Find the Vision Pro in list on the left hand side of this window.
    Select it and hit pair.
    
**Note: Remember to turn off the VPN on your mac and make AVP and your mac in the same network.**




# Error Catching

GLIBC_2.33 not found: 

    [solution](https://blog.csdn.net/huazhang_001/article/details/128828999)


AttributeError: type object 'Callable' has no attribute '_abc_registry':

    pip uninstall typing
