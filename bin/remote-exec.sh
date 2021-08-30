#!/bin/bash

D=
EPOCH=
TAG=trunk
BIN_D=$MULTIVIEW_CODE/bin
IP=
TERMINATE_COMMAND=
NOHUP=0
SSH="$(command -v autossh || echo "ssh")"

# ------------------------------------------------------------------------ tmp-d
TMPD=$(mktemp -d /tmp/$(basename $0).XXXXX)
trap cleanup EXIT
cleanup()
{
    [ -f "$TMPD/launch.out" ] \
        && echo "Terminating: " \
        && cat $TMPD/launch.out \
        && cat $TMPD/launch.out | grep terminate | dash
    rm -rf $TMPD
}

# -------------------------------------------------------------------- show help

show_help()
{
    cat - <<EOF

   Usage: $(basename $0) [OPTIONS...] -s <store> -e <epoch>

      --tag <string>     Git tag to use, default is '$TAG'
      -d <dirname>       Output directory.
      --ip <ip>          Run remotely... results are saved back locally
      --nohup            Fire and forget. The job is started nohup.

   Example:

      # 
      > $(basename $0) 

EOF
}


# ----------------------------------------------------------- parse command line

for ARG in "$@" ; do
    [ "$ARG" = "-h" ] || [ "$ARG" = "--help" ] && show_help && exit 0
done

while [ "$#" -gt "0" ] ; do
    [ "$1" = "--tag" ] && shift && TAG="$1" && continue
    [ "$1" = "-d" ] && shift && D="$1" && shift && continue
    [ "$1" = "-s" ] && shift && STORE="$1" && shift && continue
    [ "$1" = "-e" ] && shift && EPOCH="$1" && shift && continue
    [ "$1" = "--ip" ] && shift && IP="$1" && shift && continue
    [ "$1" = "--nohup" ] && NOHUP=1 && shift && continue
    echo "Unknown arguments: $*"
    exit 1
done

# ----------------------------------------------------------------------- sanity

[ "$IP" = "" ]    && echo "Must specify an IP address" && exit 1
[ "$STORE" = "" ] && echo "Must specify a store" && exit 1
[ "$EPOCH" = "" ] && echo "Must specify an epoch" && exit 1

# ----------------------------------------------- what was the remove directory?

[ "$D" = "" ] && D="${STORE}_${EPOCH}"
[ "${D: -1}" = "/" ] && D="${D:0:((${#D} - 1))}"
REMOTE_RUN_D="$HOME/runs/$D"
[ "${D:0:1}" = "/" ] && REMOTE_RUN_D="$D"

# ------------------------------------------------ is that a good manifest file?

$BIN_D/multiview-manifest.py -d "$TMPD" $STORE $EPOCH
[ "$?" != "0" ] && echo "failed to load manifest file, aborting" && exit 1

# -------------------------------------------------------------------- setup CMD

CMD_F="$TMPD/cmd.sh"
cat > "$CMD_F" <<EOF
#!/bin/bash

[ -f "\$HOME/.profile" ] && source \$HOME/.profile
[ -f "\$HOME/.bashrc" ]  && source \$HOME/.bashrc

BIN_D=\$MULTIVIEW_CODE/bin

# Make sure we're using the correct version of multiview
( cd \$MULTIVIEW_CODE ; git pull )
[ "$TAG" != "trunk" ] && ( cd \$MULTIVIEW_CODE ; git reset --hard "$TAG" )

# Create the results directory
mkdir -p "$REMOTE_RUN_D/results"

wrap()
{
    local STDOUT="$REMOTE_RUN_D/stdout.text"
    local STDERR="$REMOTE_RUN_D/stderr.text"

    { stdbuf -oL -eL "\$@" 2>&1 1>&3- | tee "\$STDERR" 1>&2 ; } 3>&1 | tee "\$STDOUT"
}

wrap \$BIN_D/setup-testcase.sh \\
    -d "$REMOTE_RUN_D/results" \\
    -m "$REMOTE_RUN_D/manifest.json"

EOF

chmod 755 "$CMD_F"

# --------------------------------------------------------- ensure we can log in

ssh-keygen -F $IP 1>/dev/null || ssh-keyscan $IP >> ~/.ssh/known_hosts

# ------------------------------------------------------------------ copy to EC2

echo "mkdir -p \"$REMOTE_RUN_D\"" | $SSH $IP
rsync -azvc "$TMPD/" "$IP:$REMOTE_RUN_D"

