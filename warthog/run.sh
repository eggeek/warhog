domains=(
  ./maps/bgmaps
  ./maps/iron
  ./maps/random10
  ./maps/starcraft
  ./maps/street
  ./maps/dao
  ./maps/maze512
  ./maps/rooms
)

maps=(
  ./testcases/maps/diag-random-512.map
  ./testcases/maps/square-random-512.map
  ./testcases/maps/maze-random.map
  ../maps/maze512/maze512-16-0.map
)

queries=(
  ./testcases/diag-random-512.query
  ./testcases/diag-random-1024.query
  ./testcases/square-random-512.query
  ./testcases/square-random-1024.query
  ./testcases/maze-random.query
  ./testcases/Berlin_0_1024.query
)

scens=(
  ./data/diag-random-512.scen
  ./data/square-random-512.scen
  ./data/maze-random.scen
  ./data/maze-random.scen
  )

algs=(
  jps
  jps-prune
  jps-prune2
  # jps-simple
  jps2
  jps2-prune
  jps2-prune2
)
data_dir="./data"
out_dir="./output"
sml_outdir="./small_output"

function gen_scen() {
  mkdir -p ${data_dir}
  for q in "${queries[@]}"; do
    mname=$(basename $q .query)
    spath="${data_dir}/${mname}.map.scen"
    if [[ ! -e $spath ]]; then
      cmd="./gen.py query2scen ${q} > ${spath}"
      echo $cmd
      eval $cmd
    fi
  done
}

function run_domain() {
  domain=$1
  dname=$(basename -- $domain)
  for mpath in `ls ${domain}/*.map`; do
    mapname=$(basename -- $mpath)
    spath="./scenarios/movingai/${dname}/${mapname}.scen"

    for alg in "${algs[@]}"; do
      outpath="${out_dir}/$alg"
      mkdir -p ${outpath}
      cmd="./bin/warthog --scen ${spath} --map ${mpath} --alg $alg > $outpath/$mapname.log"
      echo $cmd
      eval "$cmd"
    done
  done
}


function exp() {
  for dm in ${domains[@]}; do
    run_domain $dm
  done
}

function suboptcnt() {
  outdir="${out_dir}"
  fname="subopt-expd.log"
  mkdir -p ${outdir}
  if [[ -e ${outdir}/${fname} ]]; then
    rm ${outdir}/${fname}
  fi
  header=map'\t'subopt_touch'\t'tot_touch'\t'subopt_expd'\t'pruneable'\t'tot_expd'\t'scnt'\t'alg
  echo  -e "$header"> ${outdir}/${fname}
  for dm in ${domains[@]}; do
    domain=$(basename -- $dm)
    for mpath in `ls ${dm}/*.map`; do
      mapname=$(basename -- $mpath)
      spath="./scenarios/movingai/${domain}/${mapname}.scen"
      cmd="./bin/subopt_expd_exp --scen ${spath} --map ${mpath} >> ${outdir}/${fname}"
      echo $cmd
      eval $cmd
    done
  done
}

function small_exp() {
  for (( i=0; i<${#maps[@]}; i++ )); do
    mpath=${maps[$i]}
    mname=$(basename -- ${mpath})
    spath=${scens[$i]}
    for alg in ${algs[@]}; do
      mkdir -p "${sml_outdir}/${alg}"
      outpath="${sml_outdir}/${alg}/${mname}.log"
      cmd="./bin/warthog --map ${mpath} --scen ${spath} --alg ${alg} > ${outpath}"
      echo $cmd
      eval $cmd
    done
  done
}

function clean() {
  rm -f data/*.jps+
  fd -e log . "output" --no-ignore -x rm
  fd -e log . "small_output" --no-ignore -x rm
}

function gen_small() {
  cmd="./gen.py diag-map 512 > testcases/maps/diag-random-512.map"
  echo $cmd
  eval $cmd
  cmd="./gen.py square-map 512 > testcases/maps/square-random-512.map"
  echo $cmd
  eval $cmd
}

case "$1" in
  exp) exp;;
  cexp) clean && exp;;
  sexp) small_exp;;
  csexp) clean && small_exp;;
  sgen) gen_small ;;
  gen) gen_scen ;;
  sub) suboptcnt;;
  clean) clean ;; 
  *)
    echo $"Usage: $0 {exp|gen}"
    exit 1
esac
