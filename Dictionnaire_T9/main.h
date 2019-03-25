struct Dictionnaire
{
    char data;

    unsigned fin_de_mot: 1;

    struct Dictionnaire *gauche, *suite_mot, *droite;     //suite_mot est le suivant
};

struct DictionnaireCourant //Structure d'arbe des mots courant
{
    char data;

    unsigned fin_de_mot: 1;
    int occurence;
    struct DictionnaireCourant *gauche, *suite_mot, *droite;     //suite_mot est le suivant
};


typedef struct mot_courant{ //Struture permettant de manipuler facilement les mots courant et de les insérer facilement dans le fichier.
    char mot[50];   //mot le plus long  en français fait 25 lettres, 136 pour le plus long mondial et 63 pour le deuxiéme
    int occurence; //Nombre d'occurence du mot.
    struct mot_courant *suivant;
}Mots_courant;


struct Dictionnaire* newNode(char data);
struct DictionnaireCourant* newNode2(char data);
void insert(struct Dictionnaire** root, char *word);
void insert2(struct DictionnaireCourant** root_courant, char *word, int occurrence);
void traverseArbreDictionnaire(struct Dictionnaire *root, char *buffer, int depth, struct mot_courant **result);
struct mot_courant* traverseDictionnaire(struct Dictionnaire *root, const char *user_input);
void traverseArbreDictionnaire_courant(struct DictionnaireCourant *root_courant, char *buffer, int depth, struct mot_courant **result);
struct mot_courant* traverseDictionnaire_courant(struct DictionnaireCourant *root_courant, const char *user_input);
void MakeTree(struct Dictionnaire **root);
void MakeTreeCourant(struct DictionnaireCourant **root_courant);
struct mot_courant* Search(struct Dictionnaire *root, char *word, const char *user_input);
struct mot_courant* Search_courant(struct DictionnaireCourant *root_courant, char *word, const char *user_input);
void afficherMotCourant(Mots_courant *root_words);
void afficherListeMotsCourant(Mots_courant *root_words);
Mots_courant* sortedInsert(Mots_courant* head_ref, Mots_courant* new_node, bool unorganized);
struct mot_courant* list_switch( struct mot_courant* left, struct mot_courant* right );
struct mot_courant* sort( struct mot_courant* start );
Mots_courant *lireFichierMotsCourant(Mots_courant *root_words);
Mots_courant* supprimerMotCourant(Mots_courant *root, char *nom);
void ecrireFichier(Mots_courant *root);
void deleteWord();
void AddWord();
int search_dictionnary();
int menu();
