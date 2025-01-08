

# list alphabetically - more crossing lines
# namespaces="ang app cast img io obj ops pix prb ras sig sim xfm"

# list in order from lowest level to highest - fewer crossing lines
#namespaces="ang img pix ras prb io cast xfm obj ops sig app sim"

# list in order from highest level to lowest - fewer crossing lines
namespaces="sim app  sig  ops  obj xfm cast io prb  ras  pix  img  ang"


echo "digraph {"
for nmHi in $namespaces; do
	echo ""
	for nmLo in $namespaces; do
	#	echo $nmLo " using " $nmHi
		if [ "$nmLo" != "$nmHi" ]; then
		#	grep '#include' include/${nmLo}* \
		#	| grep '"'${nmHi} \
		#	| tr ':' ' ' \
		#	| awk '{print "   "  $3}'
			result=$(grep '#include' include/${nmLo}* | grep '"'${nmHi})
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
