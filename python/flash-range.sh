 for i in $(seq 0 $(($1 - 1)) );
 do
	 sn=$(printf "MP%03i" $i);
	 ./fpu-admin init $sn -180 0 --mockup;
	 ./fpu-admin flash --reuse_sn $sn $i --mockup;
 done
