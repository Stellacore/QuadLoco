

namespaces="ang cast img io obj ops pix prb ras sig sim xfm"


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
