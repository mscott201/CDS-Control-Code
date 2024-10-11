#!/bin/bash

#for the MSO7104 at imperial
# chans: which channels to read [1,2,3,4]   and 5 for math
# initialize: initalize oscilloscope and store scaling factors

chans="1 2 3 4"

# wait for any on-going operation to finish, then print 1 for "operation complete"
./ut_query "" "*WAI; *OPC?" 10000

#configure data format
./ut_write ""  ":WAVeform:FORMat BYTE"
./ut_write ""  ":WAVeform:POINts 2000000"
./ut_write ""  ":WAVeform:POINts:MODE RAW"



#equivalent to the "Single" button on the scope: capture one waveform
./ut_write ""  ":SING"
#./ut_write ""  ":DIGIT"



#wait for capture to finish, 100s timeout
./ut_query "" "*WAI; *OPC?" 10000000


#iterate over selected channels
for chan in $chans ; do

#select channel to read
./ut_write ""  ":WAVeform:SOURce CHAN$chan"

#query vertical scale
./ut_query "" :WAVeform:YINCrement? 10000

#query vertical offset 
./ut_query "" :WAVeform:YORigin?

#query vertical reference level 
./ut_query "" :WAVeform:YREFerence?

#request the data, result (binary) is written to file.dat 
./ut_save ""  ":WAVeform:DATA?" chan${chan}.dat
#data of each channel should be scaled according to (uint8_data-yref)*yinc+yori


done


#query time per sample
./ut_query "" :WAVeform:XINCrement?

#query horizontal offsets
./ut_query "" :WAVeform:XORigin?
./ut_query "" :WAVeform:XREFerence?

