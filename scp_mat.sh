#!/bin/bash
# declare an array called array and define 3 vales
array=( 20858
20867
20894
20891
20859
20866
20914
20892
20857
20865
20926
20913
20890
20856
20860
20925
20912
20893
 )
for i in "${array[@]}"
do
	echo $i
	scp yu@172.30.14.160:/home/yu/Downloads/$i/link_snrs.mat /home/xiaohui/Projects/tOR/RawData/$i/
done
