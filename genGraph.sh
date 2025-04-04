

# list alphabetically - more crossing lines
# namespaces="ang app cast img io obj ops pix prb ras sig sim xfm"

# list in order from lowest level to highest - fewer crossing lines
#namespaces="ang img pix ras prb io cast xfm obj ops sig app sim"

# list in order from highest level to lowest - fewer crossing lines
namespaces=\
"sim app  ops  obj xfm cast io prb  mat ras  pix mea  img  ang val sys"


IncDir="include/QuadLoco/"

echo "digraph {"
for nmHi in $namespaces; do
	echo ""
	for nmLo in $namespaces; do
	#	echo $nmLo " using " $nmHi
		if [ "$nmLo" != "$nmHi" ]; then
			result=$(grep '#include' ${IncDir}/${nmLo}* | grep '\/'${nmHi})
			if [ "$result" != "" ]; then
				echo "   ${nmLo} -> ${nmHi};"
			#	echo $result
			fi
		fi
	done
done
echo "}"

echo ""
echo "# E.g. to create png graphic:"
echo "# bash genGraph.sh | dot -Tpng  > foo.png"
echo ""
