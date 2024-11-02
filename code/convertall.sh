if test -z "$1"
then
    echo "You need to specify a directory."
fi


l=`ls $1/*_pre.pickle`
# echo $l
# exit

for pre in $l; do
    dat=${pre/pre/dat}
    nop=${pre/pre.pickle/}
    trj=${pre/pre/trj}
    echo "Joining $pre and $dat"
    python3 join.py $nop #2> /dev/null
    echo "Generating JSON for $trj."
    python3 sn3_to_json.py $trj #2> /dev/null

    #for mode in "joints" ; do # "polygons" "circles" "joints"
    #    for video in "novideo"; do
    #        python3 sn3_to_json.py $trj $mode $video #2> /dev/null
    #    done
    #done

done
