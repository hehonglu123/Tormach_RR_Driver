import numpy as np
import sh
try:
	sh.dpkg_query("-W","python3-robotraconteur")
except:
	sh.sudo("apt-get","update")
	sh.sudo("apt-get","install","software-properties-common")
	sh.sudo("apt-add-repository", "ppa:robotraconteur/ppa")
	sh.sudo("apt-get","update")
	sh.sudo("apt-get","install","-y","python3-robotraconteur")
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode
