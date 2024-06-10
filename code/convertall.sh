l=`ls *_trj.pickle`

for i in $l; do
    python3 sn3_to_json.py $i

done
