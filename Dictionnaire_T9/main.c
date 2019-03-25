#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "main.h"

#define MAX 50




struct Dictionnaire* newNode(char data)
{
    struct Dictionnaire* temp = (struct Dictionnaire*) malloc(sizeof( struct Dictionnaire ));
    temp->data = data;
    temp->fin_de_mot = 0;
    temp->gauche = temp->suite_mot = temp->droite = NULL;
    return temp;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct DictionnaireCourant* newNode2(char data)
{
    struct DictionnaireCourant* temp = (struct DictionnaireCourant*) malloc(sizeof( struct DictionnaireCourant ));
    temp->data = data;
    temp->occurence = 0;
    temp->fin_de_mot = 0;
    temp->gauche = temp->suite_mot = temp->droite = NULL;
    return temp;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void insert(struct Dictionnaire** root, char *word)
{
    if (!(*root))
        *root = newNode(*word);


    if ((*word) < (*root)->data) {
        insert(&((*root)->gauche), word);

    }


    else if ((*word) > (*root)->data)
        insert(&( (*root)->droite ), word);

    else
    {
        if (*(word+1))
            insert(&( (*root)->suite_mot ), word+1);


        else
            (*root)->fin_de_mot = 1;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void insert2(struct DictionnaireCourant** root_courant, char *word, int occurrence)
{
    if (!(*root_courant))
        *root_courant = newNode2(*word);


    if ((*word) < (*root_courant)->data) {
        insert2(&((*root_courant)->gauche ), word, occurrence);

    }


    else if ((*word) > (*root_courant)->data)
        insert2(&( (*root_courant)->droite ), word, occurrence);

    else
    {
        if (*(word+1))
            insert2(&( (*root_courant)->suite_mot ), word+1, occurrence);


        else {
            (*root_courant)->fin_de_mot = 1;
            (*root_courant)->occurence = occurrence;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void traverseArbreDictionnaire(struct Dictionnaire *root, char *buffer, int depth, struct mot_courant **result)
{
    if (root)
    {
        traverseArbreDictionnaire(root->gauche, buffer, depth, result);

        buffer[depth] = root->data;
        if (root->fin_de_mot)
        {
            buffer[depth+1] = '\0';
            struct mot_courant * nouveauMot;
            struct mot_courant * current;
            nouveauMot = (struct mot_courant *)malloc(sizeof(struct mot_courant));
            strcpy(nouveauMot->mot, buffer);
            nouveauMot->occurence = 0;
            nouveauMot->suivant = NULL;
            //printf( "%s indice: %d\n", nouveauMot->mot);
            if(*result == NULL){
                *result =(struct mot_courant*) malloc(sizeof(struct mot_courant));
                *result= nouveauMot;
            } else {
                current = *result;
                while (current->suivant!=NULL)
                {
                    current = current->suivant;
                }
                nouveauMot->suivant = current->suivant;
                current->suivant = nouveauMot;
            }
        }

        traverseArbreDictionnaire(root->suite_mot, buffer, depth + 1, result);

        traverseArbreDictionnaire(root->droite, buffer, depth, result);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct mot_courant* traverseDictionnaire(struct Dictionnaire *root, const char *user_input)
{
    char buffer[MAX];
    struct mot_courant *result = NULL;
    int len_user_input = strlen(user_input);
    strcpy(buffer, user_input);
    traverseArbreDictionnaire(root, buffer, len_user_input, &result);
    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void traverseArbreDictionnaire_courant(struct DictionnaireCourant *root_courant, char *buffer, int depth, struct mot_courant **result)
{
    if (root_courant)
    {
        traverseArbreDictionnaire_courant(root_courant->gauche, buffer, depth, result);

        buffer[depth] = root_courant->data;
        if (root_courant->fin_de_mot)
        {
            buffer[depth+1] = '\0';
            struct mot_courant * nouveauMot;
            struct mot_courant * current;
            nouveauMot = (struct mot_courant *)malloc(sizeof(struct mot_courant));
            strcpy(nouveauMot->mot, buffer);
            nouveauMot->occurence = root_courant->occurence;
            nouveauMot->suivant = NULL;
           // printf( "%s indice: %d\n", nouveauMot->mot);
            if(*result == NULL){
                *result =(struct mot_courant*) malloc(sizeof(struct mot_courant));
                *result= nouveauMot;
            } else {
                current = *result;
                while (current->suivant!=NULL)
                {
                    current = current->suivant;
                }
                nouveauMot->suivant = current->suivant;
                current->suivant = nouveauMot;
            }
        }

        traverseArbreDictionnaire_courant(root_courant->suite_mot, buffer, depth + 1, result);

        traverseArbreDictionnaire_courant(root_courant->droite, buffer, depth, result);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct mot_courant* traverseDictionnaire_courant(struct DictionnaireCourant *root_courant, const char *user_input)
{
    char buffer[MAX];
    struct mot_courant *result = NULL;
    int len_user_input = strlen(user_input);
    strcpy(buffer, user_input);
    traverseArbreDictionnaire_courant(root_courant, buffer, len_user_input, &result);
    return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MakeTree(struct Dictionnaire **root){

    FILE *file = fopen("dictionnaire.txt", "r");
    char word[30];
    while (feof(file) == 0){
        fscanf(file, "%s\n", word);
        //printf("mot:%s\n", word);
        insert(root, word);
    }
    fclose(file);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MakeTreeCourant(struct DictionnaireCourant **root_courant){
    FILE *file2 = fopen("mots_courant_optimized.txt", "r");
    char word2[30];
    int occurrence;
    while (fscanf(file2,"%s %d", word2, &occurrence) != EOF)
    {
        insert2(root_courant, word2, occurrence);
    }
    fclose(file2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct mot_courant* Search(struct Dictionnaire *root, char *word, const char *user_input) {
    struct mot_courant* mots_courant= NULL;
    if (!root)
        return 0;

    if (*word < (root)->data)
        return Search(root->gauche, word, user_input);

    else if (*word > (root)->data)
        return Search(root->droite, word, user_input);

    else
    {
        if (*(word+1) == '\0') {
            //printf("lettre: %s\n", word);
            mots_courant = traverseDictionnaire(root->suite_mot, user_input);
            return mots_courant;
        }

        return Search(root->suite_mot, word + 1, user_input);
    }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct mot_courant* Search_courant(struct DictionnaireCourant *root_courant, char *word, const char *user_input) {
    struct mot_courant* mots_courant= NULL;
    if (!root_courant)
        return 0;

    if (*word < (root_courant)->data)
        return Search_courant(root_courant->gauche, word, user_input);

    else if (*word > (root_courant)->data)
        return Search_courant(root_courant->droite, word, user_input);

    else
    {
        if (*(word+1) == '\0') {
            //printf("lettre: %s\n", word);
            mots_courant = traverseDictionnaire_courant(root_courant->suite_mot, user_input);
            return mots_courant;
        }

        return Search_courant(root_courant->suite_mot, word + 1, user_input);
    }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void afficherMotCourant(Mots_courant *root_words)
{
    printf("%s %d\n", root_words->mot, root_words->occurence);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void afficherListeMotsCourant(Mots_courant *root_words)
{
    Mots_courant *cpt = root_words;
    printf("Dans l'affichage de la liste! \n");
    while (cpt != NULL)
    {
        afficherMotCourant(cpt);
        cpt = cpt->suivant;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Mots_courant* sortedInsert(Mots_courant* head_ref, Mots_courant* new_node, bool unorganized)
{
    Mots_courant* current;
    if (head_ref == NULL || strcmp(head_ref->mot , new_node->mot)> 0)
    {
        if(unorganized){
            new_node->occurence += 1;
        }
        new_node->suivant = head_ref;
        head_ref = new_node;
        return head_ref;
    }
    else
    {
        /* Locate the node before the point of insertion */
        current = head_ref;
        while (current->suivant!=NULL &&
                strcmp(current->suivant->mot, new_node->mot) < 0)
        {
            current = current->suivant;
        }
        if(current->suivant!=NULL && strcmp(current->suivant->mot, new_node->mot) == 0){
            if(unorganized){
                current->suivant->occurence +=1;
            }
            return  head_ref;
        }
        new_node->suivant = current->suivant;
        current->suivant = new_node;
        if(unorganized){
            current->suivant->occurence +=1;
        }
        return head_ref;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct mot_courant* list_switch( struct mot_courant* left, struct mot_courant* right )
{
    left->suivant = right->suivant;
    right->suivant = left;
    return right;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct mot_courant* sort( struct mot_courant* start )
{
    struct mot_courant *p, *q, *top;
    int changed = 1;

    top = (struct mot_courant *)malloc(sizeof(struct mot_courant));

    top->suivant = start;
    if( start != NULL && start->suivant != NULL ) {


        while( changed ) {
            changed = 0;
            q = top;
            p = top->suivant;
            while( p->suivant != NULL ) {
                /* push bigger items down */
                if( p->occurence < p->suivant->occurence ) {
                    q->suivant = list_switch( p, p->suivant );
                    changed = 1;
                }
                q = p;
                if( p->suivant != NULL )
                    p = p->suivant;
            }
        }
    }
    p = top->suivant;
    free( top );
    return p;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Mots_courant *lireFichierMotsCourant(Mots_courant *root_words)
{
    FILE *fic;
    char nom[50];
    int montant = 0;
    Mots_courant *nouveauMot;

    fic = fopen("mots_courant_optimized.txt", "r");
    while (fscanf(fic, "%s %d", nom, &montant) != EOF)
    {
        //printf("Mot : %s, occurence: %d: \n", nom, montant);
        nouveauMot = (Mots_courant *)malloc(sizeof(Mots_courant));
        strcpy(nouveauMot->mot, nom);
        nouveauMot->occurence = montant;
        root_words = sortedInsert(root_words, nouveauMot, false);
    }
    fclose(fic);
    return root_words;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Mots_courant* supprimerMotCourant(Mots_courant *root, char *nom)
{
    bool trouve = false;
    Mots_courant *cpt = root, *cptPrec = NULL;

    while (cpt != NULL && !trouve)
    {
        if (strcmp(nom, cpt->mot) == 0) {
            trouve = true;

            // retirer le compte du chainage en faisant pointer son precedent sur son
            // suivant
            if (cptPrec == NULL)
                root = cpt->suivant;
            else
                cptPrec->suivant = cpt->suivant;
            free(cpt);
        }
        else
        {
            cptPrec = cpt;
            cpt = cpt->suivant;
        }
    }
    return root;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ecrireFichier(Mots_courant *root)
{
    FILE *fic;
    Mots_courant *cpt = root;

    fic = fopen("mots_courant_optimized.txt", "w+");

    while (cpt != NULL)
    {
        fprintf(fic, "%s %d\n", cpt->mot, cpt->occurence);
        cpt = cpt->suivant;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void deleteWord(){
    char word[110];
    printf("Saisissez le mot à supprimer : ");
    scanf("%s", word);
    Mots_courant* mots_courant= NULL;
    Mots_courant* nouveauMot = (Mots_courant *)malloc(sizeof(Mots_courant));
    strcpy(nouveauMot->mot, word);
    nouveauMot->occurence = 1;
    mots_courant = lireFichierMotsCourant(mots_courant);

    mots_courant = supprimerMotCourant(mots_courant,word);
    ecrireFichier(mots_courant);
    printf("Votre mot a été supprimé!\n\n ");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AddWord(){
    char word[110];
    printf("Saisissez le mot à ajouter : ");
    scanf("%s", word);
    Mots_courant* mots_courant= NULL;
    Mots_courant* nouveauMot = (Mots_courant *)malloc(sizeof(Mots_courant));
    strcpy(nouveauMot->mot, word);
    nouveauMot->occurence = 1;
    mots_courant = lireFichierMotsCourant(mots_courant);

    mots_courant = sortedInsert(mots_courant, nouveauMot, true);
    ecrireFichier(mots_courant);
    printf("Votre mot a été ajouté ! \n\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int search_dictionnary(){
    char saisie[50];
    char perso[50];
    while(strcmp(saisie, "quit")!=0){
        printf("Tapez les premières lettres du mot à rechercher (quit pour quitter) : ");



        scanf("%s", saisie);
        if(strcmp(saisie, "quit")==0){
            printf("A bientôt !\n");
            return 0;
        }
        struct Dictionnaire *root = NULL;
        struct DictionnaireCourant *root_courant = NULL;
        MakeTree(&root);
        MakeTreeCourant(&root_courant);
        struct mot_courant* liste_mots_dico=Search(root, saisie, saisie);
        struct mot_courant* liste_mots_courant = Search_courant(root_courant, saisie,saisie);
        struct mot_courant* tri=NULL;

        printf("Souhaitez-vous activer la recherche personnalisée ? (oui/non) : ");
        scanf("%s", perso);

        tri = sort(liste_mots_courant);
        struct mot_courant* tri_stock = tri;
        struct mot_courant* tri_stock_dico = liste_mots_dico;
        int i = 1;
        printf("Voici vos résultats :\n");
        if(strcmp(perso, "oui")==0){
            while (tri_stock != NULL && i < 4){
                printf("%s\n", tri_stock->mot);
                i++;
                tri_stock = tri_stock->suivant;
            }
        }
        while (tri_stock_dico != NULL && i < 4){
            printf("%s\n", tri_stock_dico->mot);
            tri_stock_dico = tri_stock_dico->suivant;
            i++;
        }
        printf("\nContinuer ? (oui/non) : ");
        scanf("%s",saisie);
	printf("\n");
        if(strcmp(saisie,"non") == 0){
            return 0;
        }
    }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int menu(){
    printf("1) Rechercher  un mot\n"
           "2) Ajouter un mot\n"
           "3) Supprimer un mot\n"
           "4) Quitter\n\n"
           "Entrez votre choix : ");
    int n = 0;
    scanf("%d", &n);

    switch(n){
        case 1:
            search_dictionnary();
            break;
        case 2:
           AddWord();
            break;
        case 3:
            deleteWord();
            break;
        case 4:
            printf("\n\nAu revoir !\n\n");
            return 0;
        default:{
            printf("\nSaisie incorrecte. Veuillez choisir une option dans le menu suivant : \n");
           // Menu(root2);
            break;}
    }
	printf("__________________________________________\n\n");
	menu();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main()
{
	printf("__________________________________________\n"
           "Bienvenue dans votre dictionnaire personnel !\n"
           "__________________________________________\n\n");
    menu();
    return 0;
}
