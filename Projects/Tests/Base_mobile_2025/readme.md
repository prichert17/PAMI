# Base mobile

- [Base mobile](#base-mobile)
  - [Principe de fonctionnement](#principe-de-fonctionnement)
    - [Moteurs \& encodeurs](#moteurs--encodeurs)
    - [Drivers](#drivers)
  - [Odométrie \& commande](#odométrie--commande)
    - [Odométrie](#odométrie)
    - [Commande](#commande)
  - [Commande CAN](#commande-can)

## Principe de fonctionnement 
La base mobile est une base holonome à trois roues, capable de translater dans toutes les directions sans changer d'orientation. Elle est asservie en vitesse et en position.

### Moteurs & encodeurs
Les moteurs sont des moteurs à courant continu 12V [Pololu 4751](https://www.pololu.com/product/4751) équipé d'encodeur 64 CPR et d'un motoréducteur 18,75:1. On dispose donc de 1200 pas d'encodeurs par révolution de l'arbre de sortie, ce qui permet de mesurer la position relative de l'arbre moteur et d'en dériver la vitesse de rotation de la roue.

### Drivers
Les drivers moteurs permettent de contrôler la vitesse et le sens de rotation des moteurs. Il sont basé sur le circuit intégré [DRV8256P](https://www.pololu.com/product/4039), choit pour leur linéarité. Ils sont alimentés par une source de tension continue de 12V capable de délivrer plusieurs Ampères. La vitesse du moteur est contrôlée par le rapport cyclique d'un signal PWM généré par le microcontrôleur. La direction est contrôlée par l'état d'un autre signal généré par le microcontrôleur.  

## Odométrie & commande
Afin de pouvoir correctement contrôler la vitesse et la position de la base mobile, il est primordial de connaître sa position et sa vitesse. C'est le rôle de l'odométrie.

### Odométrie
Le déplacement de la base mobile peut être calculer à partir de la rotation des roues. Une loi peut être établie entre la rotation des trois roues $s_1$, $s_2$ et $s_3$ et le déplacement en $x$, $y$ et la rotation $\theta$.
On peut établire une relation entre la vitesse des roues et la vitesse de la base mobile dans son référentiel selon 
$\begin{pmatrix}x\cr y\cr \theta\cr\end{pmatrix}=\begin{bmatrix}A_{11} &A_{12} & A_{13}\\A_{21} & A_{22} & A_{23}\\A_{31} & A_{32} & A_{33} \end{bmatrix}\times\begin{pmatrix}s_1\cr s_2\cr s_3\cr\end{pmatrix}$. Les coefficients $A_{ij}$ sont dépendant de la géométrie de la base mobile. Plus d'information sont disponible dans [cet article](https://www.researchgate.net/publication/348123505_Holonomic_Implementation_of_Three_Wheels_Omnidirectional_Mobile_Robot_using_DC_Motors) et [cet article](https://www.researchgate.net/publication/309002190_Design_Implementation_and_Validation_of_the_Three-Wheel_Holonomic_Motion_System_of_the_Assistant_Personal_Robot_APR).

### Commande
Le calcul de la commande consiste à determiner à quelle vitesse doivent tourner les roues pour faire avancer la  base mobile dans la direction voulue. On peut calculer le vecteur $\begin{pmatrix}s_1\cr s_2\cr s_3\cr\end{pmatrix}$ en utilisant l'inverse de la matrice citée précédement pour l'obtenir à partir de $\begin{pmatrix}x\cr y\cr \theta\cr\end{pmatrix}$.

## Commande CAN
Les commandes CAN sont listée ici:

*En construction*

| id | action | parameters | response |
|----|--------|------------|----------|
|0| ... | | |
|1| ... | | |
|2| ... | | |
|3| ... | | |
