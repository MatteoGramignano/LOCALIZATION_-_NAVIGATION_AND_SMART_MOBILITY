# PROGETTO LNSM

## OBUETTIVO

Il progetto ha lo scopo di valutare le prestazioni di diversi metodi di localizzazione 2D basati su misure TDOA (Time Difference of Arrival) e AOA (Angle of Arrival).
L’analisi è condotta attraverso simulazioni che impiegano un Extended Kalman Filter (EKF) con modello di moto Nearly Constant Velocity (NCV), considerando diversi dataset e condizioni di rumore.

## METODOLOGIA
- Utilizzo di misure TDOA, AOA e loro combinazione.
- Correzione dei dati AOA per passare da sistema locale a globale.
- Gestione della disponibilità variabile degli Access Points (APs).
- Stima dello stato tramite EKF.
- Refinement iterativo tramite metodo di Gauss–Newton.
- Valutazione delle prestazioni in termini di errore di posizione e velocità.

## RISULTATI PRINCIPALI
TDOA e AOA singolarmente forniscono stime affidabili solo dopo una fase iniziale di convergenza (lock-on).

La qualità della stima dipende fortemente dalla taratura della matrice di covarianza: una scelta non ottimale rallenta la convergenza dell’EKF.

L’uso di iterazioni multiple del metodo Gauss–Newton migliora significativamente l’accuratezza.

La combinazione TDOA + AOA rappresenta la soluzione più performante, riducendo l’errore di localizzazione.

Esiste un trade-off tra accuratezza e costo computazionale: per applicazioni real-time è possibile limitare il numero di iterazioni mantenendo buone prestazioni.

## CONSIDERAZIONI FINALI
- Il modello NCV risulta adeguato entro determinati limiti dinamici del sistema.
- L’accuratezza dipende sia dalla qualità delle misure sia dalla disponibilità degli APs.
- L’integrazione di diverse tipologie di misura è fondamentale per ottenere risultati robusti.
- L’approccio proposto è applicabile in scenari reali, con opportuni compromessi tra precisione e tempo di calcolo.

Questo progetto dimostra come tecniche di fusione sensoriale e ottimizzazione iterativa possano migliorare significativamente la localizzazione in ambienti realistici.
