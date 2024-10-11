#!/bin/bash
#Need to compile the code that converts the .dat file to .txt
#be careful to have the correct buffer size for your data
#peak detect needs twice the number of slots as we have data point(first entry is a reference/copy)
g++ -ggdb -W -lusb `root-config --cflags --glibs --ldflags` --permissive bin2txt_4channels.c -o bin2txt_4channels


#this was for one single measurement
#for ((i=$p; i<=$p; i++))
    #do
    for (( c=0; c<=500; c++ ))
    do
        a=$(./get_mso_4.sh) 
        y_1=$(echo $a | cut -d " " -f3)
        y_2=$(echo $a | cut -d " " -f4)
        y_3=$(echo $a | cut -d " " -f5)
        y2_1=$(echo $a | cut -d " " -f6)
        y2_2=$(echo $a | cut -d " " -f7)
        y2_3=$(echo $a | cut -d " " -f8)
        y3_1=$(echo $a | cut -d " " -f9)
        y3_2=$(echo $a | cut -d " " -f10)
        y3_3=$(echo $a | cut -d " " -f11)
        y4_1=$(echo $a | cut -d " " -f12)
        y4_2=$(echo $a | cut -d " " -f13)
        y4_3=$(echo $a | cut -d " " -f14)
        #echo $c $y_1 $y_2 $y_3 >> /home/a/WCTE540/Bsci_Project_T1_2023/laserOn_data/laserdiffuser_3PMTs/peak_detect/scaling_chan1_2-6kV_$i.txt
        #echo $c $y2_1 $y2_2 $y2_3 >> /home/a/WCTE540/Bsci_Project_T1_2023/laserOn_data/laserdiffuser_3PMTs/peak_detect/scaling_chan2_2-6kV_$i.txt
        #echo $c $y3_1 $y3_2 $y3_3 >> /home/a/WCTE540/Bsci_Project_T1_2023/laserOn_data/laserdiffuser_3PMTs/peak_detect/scaling_chan3_2-6kV_$i.txt
        #./script_octave.sh
        #mv Dark_rate.png /home/a/WCTE218/Bsci_Project_T1_2023/Dark_Rate_test_15-04_lights_off/Dark_rate_$c.png
        mv -f chan1.dat /home/a/WCTE540/Bsci_Project_T1_2023/Data/Storage_dat_files/chan1_$c.dat
        mv -f chan2.dat /home/a/WCTE540/Bsci_Project_T1_2023/Data/Storage_dat_files/chan2_$c.dat
        mv -f chan3.dat /home/a/WCTE540/Bsci_Project_T1_2023/Data/Storage_dat_files/chan3_$c.dat
        mv -f chan4.dat /home/a/WCTE540/Bsci_Project_T1_2023/Data/Storage_dat_files/chan4_$c.dat
	echo "We have moved the file "$c""
	echo $y_1 $y_2 $y_3 $y2_1 $y2_2 $y2_3 $y3_1 $y3_2 $y3_3  $y4_1 $y4_2 $y4_3 
        #mv chan3.dat /home/a/WCTE218/Bsci_Project_T1_2023/Dark_Rate_test_15-04_lights_off/chan3_$c.dat
        #./bin2txt /home/a/WCTE540/Bsci_Project_T1_2023/laserOn_data/Monitor_PMT/peak_detect/chan1_test_$c.dat > /home/a/WCTE540/Bsci_Project_T1_2023/laserOn_data/Monitor_PMT/peak_detect/ADC/chan1_test_$c.txt
        ./bin2txt_4channels /home/a/WCTE540/Bsci_Project_T1_2023/Data/Storage_dat_files/chan1_$c.dat /home/a/WCTE540/Bsci_Project_T1_2023/Data/Storage_dat_files/chan2_$c.dat /home/a/WCTE540/Bsci_Project_T1_2023/Data/Storage_dat_files/chan3_$c.dat /home/a/WCTE540/Bsci_Project_T1_2023/Data/Storage_dat_files/chan4_$c.dat $y_1 $y_2 $y_3 $y2_1 $y2_2 $y2_3 $y3_1 $y3_2 $y3_3 $y4_1 $y4_2 $y4_3 > /home/a/WCTE540/Bsci_Project_T1_2023/Data/Storage_txt_files/test_$c.txt
	#next line: do not change the mv initial location, it's hard coded in bin2txt, needs to be square and tts.
	mv /home/a/WCTE540/Bsci_Project_T1_2023/out.root /home/a/WCTE540/Bsci_Project_T1_2023/Data/root_files/Test_$c.root
        echo $c
    done
#done
