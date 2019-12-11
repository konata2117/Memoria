import re 

class Parseo:
	def leer_archivo(self,nombre):
		with open(nombre,"r") as a:
			return a.read()

	def parseo(self,nombre):
		s= self.leer_archivo(nombre)
		ll= s.split('\n')
		
		acciones ={}
		parametro = {}
		acciones['action']=[]
		parametro['parametro'] = []
		patron = ('action:')
		pal = re.compile(patron)
	
		for i in range(len(ll)):
			q = ll[i]
			j= q.find(" parameters: ")
			if j >= 0:
					t = q.split(":")
					parametro['parametro'].append(t[1]) 
			if pal.match(ll[i]) != None:
				t= ll[i].split(': ')
				acciones['action'].append(t[1])
						
		print acciones
		print len(acciones)
		print parametro
		print parametro['parametro'][0]
		patron1 = re.compile(r'\w\w+')
		para = []
		
		for i in range(len(acciones['action'])):
			qq = parametro['parametro'][i]
			
			palabra = patron1.findall(qq)
			para.append(palabra)
			print palabra
			print len(palabra)
		#print para[1][0]
		return acciones, para 


def main():
		par = Parseo()
		rt , t = par.parseo('elplan.txt')
		print "acciones ", rt
		print "parametros", t
if __name__ == '__main__':
	main()