#!/bin/bash
domains=(
dao
bgmaps
starcraft
mazes
random10
random20
random30
random40
rooms
street
)
algs=(
jps-prune
jps
)

function run() {
  for domain in "${domains[@]}"; do
    mapnames=(`ls "./maps/${domain}"`)
    for map in "${mapnames[@]}"; do
      for alg in "${algs[@]}"; do
        odir="./output/$alg/$domain"
        mkdir -p $odir
        spath="./scenarios/movingai/${domain}/${map}.scen"
        mpath="./maps/${domain}/${map}"
        opath="$odir/$map.csv"
        cmd="./bin/warthog --checkopt --scen $spath --map $mpath --alg $alg > $opath"
        echo $cmd
        eval $cmd
      done
    done
  done
}

run
