
class PLAN:
	def planes(self, doc1, doc2, doc3):
		f = open(doc1, 'r')
		r = open(doc2, 'w')
		t = open(doc3, 'r')
		mensaje = f.read()
		lis = mensaje
		lis = lis.split("\n")
		print lis

		mensajes = t.read()
		print mensajes
		lisa = mensajes
		lisa = lisa.split("\n")
		lisa.pop(len(lisa) - 1)
		print lisa
		r.write("(define (problem p8) "  + "\n" + "(:domain blocksworld)" +"\n"+ "(:objects " )
		for i in range(len(lis)):
			r.write(" " + lis[i] + " ")

		r.write(")" + "\n")
		r.write("(:init ")
		for i in range(len(lisa) ):
			r.write("(onTable " + lisa[i] + ") " )
			r.write ("(clear " + lisa[i] + ") " )
			r.write("(equal " + lisa[i] + " " + lisa[i] + ") " + "\n")
		r.write(")" + "\n")

		r.write("(:goal (and ") 
		for i in range(len(lis)-1):
	 		r.write("(on "+ lis[i + 1] + " " + lis[i ] + ")")

		r.write(")))")
		print mensaje
		f.close()
		r.close()
		return lis

if __name__ == '__main__':
	plaan = PLAN()
	plas = plaan.planes("doc.txt","p8.pddl","inicio.txt")
	print plas