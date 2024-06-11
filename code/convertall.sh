l=`ls *_pre.pickle`

for pre in $l; do
    dat=${pre/pre/dat}
    nop=${pre/pre.pickle/}
    trj=${pre/pre/trj}
    echo "Joining $pre and $dat"
    pytho3 join.py $nop 2> /dev/null
    echo "Generating JSON for $trj."
    python3 sn3_to_json.py $trj 2> /dev/null


done
