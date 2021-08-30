#!/bin/bash

show_help()
{
    EXEC="$(basename $0)"
    
    cat <<EOF

   Usage: $EXEC <version1> <version2>

   Returns '0' => equals
           '1' => <version1> greater <version2>
           '2' => <version1> less <version2>

   Examples:

      $EXEC  3.18.1   3.9.4

EOF
}

vercomp()
{
    if [[ $1 == $2 ]]
    then
        return 0
    fi
    local IFS=.
    local i ver1=($1) ver2=($2)
    # fill empty fields in ver1 with zeros
    for ((i=${#ver1[@]}; i<${#ver2[@]}; i++))
    do
        ver1[i]=0
    done
    for ((i=0; i<${#ver1[@]}; i++))
    do
        if [[ -z ${ver2[i]} ]]
        then
            # fill empty fields in ver2 with zeros
            ver2[i]=0
        fi
        if ((10#${ver1[i]} > 10#${ver2[i]}))
        then
            return 1
        fi
        if ((10#${ver1[i]} < 10#${ver2[i]}))
        then
            return 2
        fi
    done
    return 0
}

testvercomp()
{
    vercomp $1 $2
    case $? in
        0) op='=';;
        1) op='>';;
        2) op='<';;
    esac
    if [[ $op != $3 ]]
    then
        echo "FAIL: Expected '$3', Actual '$op', Arg1 '$1', Arg2 '$2'"
    else
        echo "Pass: '$1 $op $2'"
    fi
}

run_tests()
{
    # Run tests
    # argument table format:
    # testarg1   testarg2     expected_relationship
    echo "The following tests should pass"
    while read -r test
    do
        testvercomp $test
    done << EOF
1            1            =
2.1          2.2          <
3.0.4.10     3.0.4.2      >
4.08         4.08.01      <
3.2.1.9.8144 3.2          >
3.2          3.2.1.9.8144 <
1.2          2.1          <
2.1          1.2          >
5.6.7        5.6.7        =
1.01.1       1.1.1        =
1.1.1        1.01.1       =
1            1.0          =
1.0          1            =
1.0.2.0      1.0.2        =
1..0         1.0          =
1.0          1..0         =
EOF

    echo "The following test should fail (test the tester)"
    testvercomp 3.18.1 3.9.4 '<'
}

for ARG in "$@" ; do
    [ "$ARG" = "-h" ] || [ "$ARG" = "--help" ] && show_help && exit 0
done

[ "$#" != "2" ] && echo "expected 2 arguments, type -h for help" && exit 1


vercomp "$1" "$2"
echo $?
exit 0

