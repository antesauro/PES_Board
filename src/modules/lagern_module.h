typedef struct
{
	int postion;
    bool belegt;
	int farbe;
} paklilager;

class LagernModule
{
public:
	LagernModule();

private:
	paklilager rot;
	paklilager blau;
	paklilager gelb;
	paklilager gruen;
};
