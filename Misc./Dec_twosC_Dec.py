# Takes in a +decimal from a binary which wasn't interpreted as a two's complement
# Returns the decimal as a two's complement interpretation of that +decimal

# 16-bit Positive Decimal to Two's Complemnt to +/- 15-bit Decimal
def DecodeWord(Decimal, Bits): 

	#Convert decimal to binary string
	bDec = bin(Decimal).replace("0b","")

	# If len binary < span, the MSB is 0 so it is a positive number, return the decimal
	if len(bDec)<Bits:
		return Decimal

	# If len == Bits, the 16th position contains a 1, so it is a negative number
	# Convert the remaining 15 bits (discluding the MSB) into a negative number
	elif len(bDec)==Bits:
		nDec = int(bDec[1::],2)-(2**(Bits-1))
		if nDec == -65536:
			return "DROP"
		return nDec

	elif int(bDec,2)==2**Bits:
		return 0

	else:
		return "ERROR"




Decimal = int(input("Enter a Decimal Value: "))
Bits = int(input("How many Binary-Bits is the Number Spanning: "))
print(DecodeWord(Decimal, Bits))

# print(f"Decimal Value: {x}, 16-bit Binary: {y}, 2's Complement in Decimal: {z}.")