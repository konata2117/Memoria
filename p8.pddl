(define (problem p8) 
(:domain blocksworld)
(:objects  VERDE  ROJO )
(:init (onTable VERDE) (clear VERDE) (equal VERDE VERDE) 
(onTable ROJO) (clear ROJO) (equal ROJO ROJO) 

)
(:goal (and (on ROJO VERDE))))
