(define (problem p8) 
(:domain blocksworld)
(:objects  VERDE  AZUL )
(:init (onTable VERDE) (clear VERDE) (equal VERDE VERDE) 
(onTable AZUL) (clear AZUL) (equal AZUL AZUL) 
)
(:goal (and (on AZUL VERDE))))