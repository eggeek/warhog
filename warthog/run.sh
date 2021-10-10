maps=(
  ./maps/iron/scene_sp_endmaps.map
  ./endmaps_2.map
  ./endmaps_3.map
  ./endmaps_4.map
  ./endmaps_5.map
  ./endmaps_6.map
  ./endmaps_7.map
  ./endmaps_8.map
)

scens=(
  ./scenarios/movingai/iron/scene_sp_endmaps.map.scen
  ./endmaps_2.map.scen
  ./endmaps_3.map.scen
  ./endmaps_4.map.scen
  ./endmaps_5.map.scen
  ./endmaps_6.map.scen
  ./endmaps_7.map.scen
  ./endmaps_8.map.scen
)

algs=(rect)

for (( i=0; i<${#maps[@]}; i++ )); do
  mpath=${maps[$i]}
  spath=${scens[$i]}
  mapname=$(basename -- $mpath)
  for alg in "${algs[@]}"; do
    outpath="./output/$alg/"
    cmd="./bin/warthog --scen ${spath} --map ${mpath} --alg $alg > $outpath/$mapname.log"
    echo $cmd
    eval "$cmd"
  done
done
