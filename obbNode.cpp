#include <calcUtils.h>
#include <obbNode.h>
#include <obbStruct.h>

const double q1[91*4]={
    double(1.0),double(0.0),double(0.0),double(0.0),
    double(0.99384415149689),double(-0.0061558117158711),double(0.078217260539532),double(0.078217305243015),
    double(0.96937239170074),double(-0.018315907567739),double(0.23272576928139),double(0.076291389763355),
    double(0.92103153467178),double(-0.030024981126189),double(0.38150382041931),double(0.072486810386181),
    double(0.85001176595688),double(-0.040994759649038),double(0.52088803052902),double(0.066897414624691),
    double(0.75806188583374),double(-0.050955086946487),double(0.64744603633881),double(0.059660792350769),
    double(0.94550329446793),double(-0.05449678376317),double(0.22699527442455),double(0.22699521481991),
    double(0.89835268259048),double(-0.089335732161999),double(0.37210986018181),double(0.21567536890507),
    double(0.82908165454865),double(-0.1219749674201),double(0.5080618262291),double(0.19904483854771),
    double(0.73939591646194),double(-0.15161071717739),double(0.63150370121002),double(0.17751318216324),
    double(0.92103147506714),double(-0.03002499602735),double(0.072486750781536),double(0.38150382041931),
    double(0.89835262298584),double(-0.089335650205612),double(0.21567541360855),double(0.3721099793911),
    double(0.85355335474014),double(-0.14644660055637),double(0.35355341434479),double(0.3535535633564),
    double(0.78773677349091),double(-0.19995146989822),double(0.48272582888603),double(0.3262914121151),
    double(0.70252358913422),double(-0.2485329657793),double(0.60001170635223),double(0.29099488258362),
    double(0.82908165454865),double(-0.12197487056255),double(0.19904492795467),double(0.50806188583374),
    double(0.78773683309555),double(-0.19995151460171),double(0.32629129290581),double(0.48272570967674),
    double(0.72699528932571),double(-0.27300474047661),double(0.44550329446793),double(0.44550329446793),
    double(0.6483525633812),double(-0.33933570981026),double(0.5537456870079),double(0.39731112122536),
    double(0.75806188583374),double(-0.05095511302352),double(0.059660773724318),double(0.64744603633881),
    double(0.73939591646194),double(-0.15161070227623),double(0.17751322686672),double(0.63150376081467),
    double(0.70252358913422),double(-0.24853309988976),double(0.29099479317665),double(0.60001170635223),
    double(0.64835274219513),double(-0.33933570981026),double(0.39731100201607),double(0.55374538898468),
    double(0.57821726799011),double(-0.42178285121918),double(0.49384421110153),double(0.49384415149689),
    double(0.63150358200073),double(-0.17751328647137),double(0.15161070227623),double(0.73939609527588),
    double(0.60001170635223),double(-0.29099473357201),double(0.24853305518627),double(0.70252352952957),
    double(0.55374562740326),double(-0.39731097221375),double(0.33933568000793),double(0.64835262298584),
    double(0.49384415149689),double(-0.49384415149689),double(0.42178285121918),double(0.57821726799011),
    double(0.52088797092438),double(-0.066897362470627),double(0.040994789451361),double(0.85001170635223),
    double(0.50806170701981),double(-0.199044957757),double(0.12197488546371),double(0.82908183336258),
    double(0.4827256500721),double(-0.32629129290581),double(0.19995157420635),double(0.7877368927002),
    double(0.4455032646656),double(-0.44550320506096),double(0.27300474047661),double(0.72699534893036),
    double(0.39731097221375),double(-0.55374550819397),double(0.33933565020561),double(0.64835268259048),
    double(0.37210977077484),double(-0.21567541360855),double(0.089335687458515),double(0.89835274219513),
    double(0.35355335474014),double(-0.35355344414711),double(0.14644664525986),double(0.85355341434479),
    double(0.32629126310349),double(-0.48272567987442),double(0.19995155930519),double(0.78773683309555),
    double(0.29099473357201),double(-0.60001170635223),double(0.24853302538395),double(0.70252352952957),
    double(0.23272578418255),double(-0.076291278004646),double(0.018315944820642),double(0.96937245130539),
    double(0.22699522972107),double(-0.22699525952339),double(0.054496757686138),double(0.94550329446793),
    double(0.21567545831203),double(-0.37210991978645),double(0.08933574706316),double(0.89835262298584),
    double(0.19904497265816),double(-0.50806188583374),double(0.12197491526604),double(0.82908165454865),
    double(0.17751327157021),double(-0.63150376081467),double(0.15161073207855),double(0.73939591646194),
    double(0.076291263103485),double(-0.23272579908371),double(0.018315916880965),double(0.96937245130539),
    double(0.072486750781536),double(-0.38150373101234),double(0.03002498857677),double(0.92103159427643),
    double(0.066897392272949),double(-0.52088785171509),double(0.040994752198458),double(0.85001176595688),
    double(0.059660766273737),double(-0.64744603633881),double(0.05095511674881),double(0.75806188583374),
    double(-0.078217208385468),double(-0.078217223286629),double(-0.006155826151371),double(0.99384421110153),
    double(-0.076291233301163),double(-0.23272578418255),double(-0.018315909430385),double(0.96937245130539),
    double(-0.072486698627472),double(-0.38150376081467),double(-0.030024975538254),double(0.92103159427643),
    double(-0.066897362470627),double(-0.52088785171509),double(-0.040994733572006),double(0.85001182556152),
    double(-0.059660736471415),double(-0.64744603633881),double(-0.050955090671778),double(0.75806194543839),
    double(-0.22699531912804),double(-0.22699527442455),double(-0.054496750235558),double(0.94550323486328),
    double(-0.21567545831203),double(-0.37210991978645),double(-0.089335672557354),double(0.89835268259048),
    double(-0.19904491305351),double(-0.50806188583374),double(-0.12197485566139),double(0.82908165454865),
    double(-0.17751330137253),double(-0.63150376081467),double(-0.15161062777042),double(0.7393958568573),
    double(-0.38150364160538),double(-0.072486743330956),double(-0.030024966225028),double(0.92103159427643),
    double(-0.37210974097252),double(-0.21567544341087),double(-0.089335672557354),double(0.89835274219513),
    double(-0.3535532951355),double(-0.35355341434479),double(-0.1464466303587),double(0.85355347394943),
    double(-0.32629123330116),double(-0.48272570967674),double(-0.19995157420635),double(0.7877368927002),
    double(-0.29099464416504),double(-0.60001176595688),double(-0.24853302538395),double(0.70252358913422),
    double(0.50806170701981),double(0.199044957757),double(0.1219748929143),double(-0.82908171415329),
    double(-0.48272567987442),double(-0.32629126310349),double(-0.19995151460171),double(0.78773683309555),
    double(-0.44550320506096),double(-0.44550329446793),double(-0.27300471067429),double(0.72699534893036),
    double(-0.39731097221375),double(-0.55374556779861),double(-0.33933565020561),double(0.64835274219513),
    double(0.64744591712952),double(0.059660769999027),double(0.050955083221197),double(-0.75806200504303),
    double(0.63150358200073),double(0.17751330137253),double(0.15161070227623),double(-0.73939609527588),
    double(0.60001164674759),double(0.29099479317665),double(0.24853308498859),double(-0.70252364873886),
    double(0.55374544858932),double(0.39731103181839),double(0.3393357694149),double(-0.64835274219513),
    double(-0.49384412169456),double(-0.49384415149689),double(-0.42178279161453),double(0.57821726799011),
    double(0.73939591646194),double(0.15161064267159),double(0.17751328647137),double(-0.63150376081467),
    double(0.70252352952957),double(0.24853302538395),double(0.29099476337433),double(-0.60001170635223),
    double(0.64835262298584),double(0.3393357694149),double(0.39731109142303),double(-0.55374556779861),
    double(0.57821714878082),double(0.42178282141685),double(0.49384421110153),double(-0.49384421110153),
    double(0.85001182556152),double(0.040994759649038),double(0.066897377371788),double(-0.52088791131973),
    double(0.82908165454865),double(0.12197490036488),double(0.19904498755932),double(-0.50806188583374),
    double(0.78773683309555),double(0.19995154440403),double(0.32629129290581),double(-0.48272573947906),
    double(0.72699528932571),double(0.27300474047661),double(0.44550332427025),double(-0.4455032646656),
    double(0.64835268259048),double(0.33933568000793),double(0.55374556779861),double(-0.39731100201607),
    double(0.89835262298584),double(0.089335709810257),double(0.21567544341087),double(-0.37210991978645),
    double(0.85355341434479),double(0.14644664525986),double(0.35355338454247),double(-0.35355335474014),
    double(0.78773677349091),double(0.19995158910751),double(0.48272570967674),double(-0.32629129290581),
    double(0.70252346992493),double(0.24853307008743),double(0.60001182556152),double(-0.29099479317665),
    double(0.96937245130539),double(0.018315913155675),double(0.076291263103485),double(-0.2327257245779),
    double(0.94550323486328),double(0.054496750235558),double(0.22699530422688),double(-0.22699522972107),
    double(0.89835268259048),double(0.089335694909096),double(0.37210991978645),double(-0.21567544341087),
    double(0.82908165454865),double(0.12197487801313),double(0.50806188583374),double(-0.19904491305351),
    double(0.73939591646194),double(0.15161064267159),double(0.63150370121002),double(-0.17751325666904),
    double(0.96937245130539),double(0.018315905705094),double(0.23272575438023),double(-0.076291218400002),
    double(0.92103153467178),double(0.030024981126189),double(0.38150382041931),double(-0.072486713528633),
    double(0.85001182556152),double(0.040994741022587),double(0.52088791131973),double(-0.066897377371788),
    double(0.75806188583374),double(0.050955094397068),double(0.64744609594345),double(-0.059660740196705)
};

const double q2[36*4]={
    double(0.98432183265686),double(-0.0054544419981539),double(0.030933555215597),double(0.17356246709824),
    double(0.98345810174942),double(-0.0090880254283547),double(0.051540859043598),double(0.17341016232967),
    double(0.98216301202774),double(-0.012717707082629),double(0.072125561535358),double(0.17318181693554),
    double(0.98043715953827),double(-0.016341743990779),double(0.092678628861904),double(0.17287746071815),
    double(0.86559802293777),double(-0.015705356374383),double(0.027202559635043),double(0.49975338578224),
    double(0.86483854055405),double(-0.02616797760129),double(0.045324314385653),double(0.49931478500366),
    double(0.86369961500168),double(-0.036619070917368),double(0.063426144421101),double(0.49865740537643),
    double(0.86218190193176),double(-0.047054179012775),double(0.081500194966793),double(0.4977810382843),
    double(0.64247047901154),double(-0.024062059819698),double(0.020190495997667),double(0.76566648483276),
    double(0.64190673828125),double(-0.040091678500175),double(0.033640909940004),double(0.76499456167221),
    double(0.64106142520905),double(-0.056103721261024),double(0.047076646238565),double(0.76398718357086),
    double(0.63993483781815),double(-0.072091154754162),double(0.060491673648357),double(0.76264482736588),
    double(0.34185141324997),double(-0.029516492038965),double(0.010743148624897),double(0.939228951931),
    double(0.34155142307281),double(-0.049179725348949),double(0.017899960279465),double(0.93840479850769),
    double(0.34110167622566),double(-0.068821407854557),double(0.025048969313502),double(0.93716907501221),
    double(0.34050220251083),double(-0.088432893157005),double(0.032186944037676),double(0.93552225828171),
    double(-2.9802322387695e-08),double(-0.031410802155733),double(2.0489096641541e-08),double(0.99950659275055),
    double(-2.9802318834982e-08),double(-0.052335970103741),double(-1.8626449271864e-09),double(0.99862957000732),
    double(-2.9802325940409e-08),double(-0.073238231241703),double(1.8626453268666e-08),double(0.99731451272964),
    double(-1.1920928955078e-07),double(-0.094108313322067),double(-3.7252907425511e-09),double(0.99556201696396),
    double(-0.34185135364532),double(-0.029516480863094),double(-0.010743118822575),double(0.93922901153564),
    double(-0.34155142307281),double(-0.04917972907424),double(-0.017899954691529),double(0.93840485811234),
    double(-0.34110155701637),double(-0.068821392953396),double(-0.025048937648535),double(0.93716907501221),
    double(-0.34050226211548),double(-0.088432945311069),double(-0.032186958938837),double(0.93552225828171),
    double(0.64247035980225),double(0.024062030017376),double(0.02019046805799),double(-0.76566654443741),
    double(0.64190655946732),double(0.040091685950756),double(0.033640891313553),double(-0.76499480009079),
    double(0.64106148481369),double(0.056103706359863),double(0.047076635062695),double(-0.76398718357086),
    double(0.63993483781815),double(0.072091184556484),double(0.060491684824228),double(-0.76264482736588),
    double(0.86559802293777),double(0.015705425292253),double(0.027202542871237),double(-0.49975338578224),
    double(0.8648384809494),double(0.026167996227741),double(0.04532428830862),double(-0.4993149638176),
    double(0.86369961500168),double(0.036619134247303),double(0.063426159322262),double(-0.49865740537643),
    double(0.86218190193176),double(0.047054175287485),double(0.081500202417374),double(-0.49778109788895),
    double(0.98432177305222),double(0.0054544052109122),double(0.030933560803533),double(-0.17356267571449),
    double(0.98345810174942),double(0.0090880719944835),double(0.051540851593018),double(-0.17341040074825),
    double(0.98216301202774),double(0.012717668898404),double(0.072125554084778),double(-0.17318204045296),
    double(0.98043709993362),double(0.016341753304005),double(0.092678636312485),double(-0.17287777364254)
};

const double q3[10*4]={
    double(0.95105654001236),double(0.0),double(0.0),double(-0.30901706218719),
    double(0.9723699092865),double(0.0),double(0.0),double(-0.23344537615776),
    double(0.98768836259842),double(0.0),double(0.0),double(-0.1564345061779),
    double(0.99691736698151),double(0.0),double(0.0),double(-0.078459121286869),
    double(0.99691736698151),double(0.0),double(0.0),double(0.078459121286869),
    double(0.98768836259842),double(0.0),double(0.0),double(0.1564345061779),
    double(0.9723699092865),double(0.0),double(0.0),double(0.23344537615776),
    double(0.95105654001236),double(0.0),double(0.0),double(0.30901706218719),
    double(0.9238795042038),double(0.0),double(0.0),double(0.38268345594406),
    double(0.89100652933121),double(0.0),double(0.0),double(0.45399054884911)
};

const double q4[10*4]={
    double(0.9993908405304),double(0.0),double(0.0),double(-0.034899495542049),
    double(0.99965733289719),double(0.0),double(0.0),double(-0.026176949962974),
    double(0.9998477101326),double(0.0),double(0.0),double(-0.017452405765653),
    double(0.99996191263199),double(0.0),double(0.0),double(-0.0087265353649855),
    double(0.99996191263199),double(0.0),double(0.0),double(0.0087265353649855),
    double(0.9998477101326),double(0.0),double(0.0),double(0.017452405765653),
    double(0.99965733289719),double(0.0),double(0.0),double(0.026176949962974),
    double(0.9993908405304),double(0.0),double(0.0),double(0.034899495542049),
    double(0.99904823303223),double(0.0),double(0.0),double(0.043619386851788),
    double(0.99862951040268),double(0.0),double(0.0),double(0.05233595892787)
};

CObbNode::CObbNode()
{
    obbNodes=nullptr;
    leafTris=nullptr;
}

CObbNode::CObbNode(const std::vector<double>& allVertices,const std::vector<int>& allTriangles,const std::vector<int>& triIndices,size_t maxTriCnt)
{
    boxM=getNaturalFrame(allVertices,allTriangles,triIndices,boxHs);
    if (triIndices.size()>maxTriCnt)
    {
        leafTris=nullptr;
        std::vector<int> triIndices1;
        std::vector<int> triIndices2;
        splitTriangles(allVertices,allTriangles,triIndices,triIndices1,triIndices2);
        obbNodes=new CObbNode* [2];
        obbNodes[0]=new CObbNode(allVertices,allTriangles,triIndices1,maxTriCnt);
        obbNodes[1]=new CObbNode(allVertices,allTriangles,triIndices2,maxTriCnt);
    }
    else
    {
        obbNodes=nullptr;
        leafTris=new std::vector<int>(triIndices);
    }
}

CObbNode::~CObbNode()
{
    if (obbNodes!=nullptr)
    {
        delete obbNodes[0];
        delete obbNodes[1];
        delete[] obbNodes;
    }
    delete leafTris;
}

CObbNode* CObbNode::copyYourself() const
{
    CObbNode* theCopy=new CObbNode();
    theCopy->boxHs=boxHs;
    theCopy->boxM=boxM;
    if (leafTris==nullptr)
    {
        theCopy->obbNodes=new CObbNode* [2];
        theCopy->obbNodes[0]=obbNodes[0]->copyYourself();
        theCopy->obbNodes[1]=obbNodes[1]->copyYourself();
    }
    else
        theCopy->leafTris=new std::vector<int>(leafTris[0]);
    return(theCopy);
}

void CObbNode::scaleYourself(double f)
{
    boxM.X=boxM.X*f;
    boxHs=boxHs*f;
    if (leafTris==nullptr)
    {
        obbNodes[0]->scaleYourself(f);
        obbNodes[1]->scaleYourself(f);
    }
}

void CObbNode::serialize(std::vector<unsigned char>& data) const
{
    C7Vector tr(boxM);
    pushData(data,&tr.X(0),sizeof(double));
    pushData(data,&tr.X(1),sizeof(double));
    pushData(data,&tr.X(2),sizeof(double));
    pushData(data,&tr.Q(0),sizeof(double));
    pushData(data,&tr.Q(1),sizeof(double));
    pushData(data,&tr.Q(2),sizeof(double));
    pushData(data,&tr.Q(3),sizeof(double));
    pushData(data,&boxHs(0),sizeof(double));
    pushData(data,&boxHs(1),sizeof(double));
    pushData(data,&boxHs(2),sizeof(double));
    if (leafTris!=nullptr)
    {
        data.push_back(0);
        unsigned short w=static_cast<unsigned short>(leafTris->size());
        pushData(data,&w,sizeof(unsigned short));
        for (size_t i=0;i<leafTris->size();i++)
            pushData(data,&leafTris->at(i),sizeof(int));
    }
    else
    {
        data.push_back(1);
        obbNodes[0]->serialize(data);
        obbNodes[1]->serialize(data);
    }
}

void CObbNode::deserialize(const unsigned char* data,int& pos)
{
    C7Vector tr;
    tr.X(0)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
    tr.X(1)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
    tr.X(2)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
    tr.Q(0)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
    tr.Q(1)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
    tr.Q(2)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
    tr.Q(3)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
    boxM=tr.getMatrix();
    boxHs(0)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
    boxHs(1)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
    boxHs(2)=(reinterpret_cast<const double*>(data+pos))[0];pos+=sizeof(double);
    unsigned char leafn=data[pos++];
    if (leafn==0)
    {
        size_t triCnt=(reinterpret_cast<const unsigned short*>(data+pos))[0];pos+=sizeof(unsigned short);
        leafTris=new std::vector<int>;
        for (size_t i=0;i<triCnt;i++)
        {
            leafTris->push_back((reinterpret_cast<const int*>(data+pos))[0]);
            pos+=sizeof(int);
        }
    }
    else
    {
        obbNodes=new CObbNode* [2];
        obbNodes[0]=new CObbNode();
        obbNodes[0]->deserialize(data,pos);
        obbNodes[1]=new CObbNode();
        obbNodes[1]->deserialize(data,pos);
    }
}

void CObbNode::splitTriangles(const std::vector<double>& allVertices,const std::vector<int>& allTriangles,const std::vector<int>& triIndices,std::vector<int>& triIndicesGroup1,std::vector<int>& triIndicesGroup2)
{
    C3Vector hs(boxHs);

    for (size_t loop=0;loop<3;loop++)
    {
        int bestAxis=0;
        double bestAxisS=0.0;
        for (size_t i=0;i<3;i++)
        {
            if (hs(i)>bestAxisS)
            {
                bestAxisS=hs(i);
                hs(i)=0.0;
                bestAxis=int(i);
            }
        }
        triIndicesGroup1.clear();
        triIndicesGroup2.clear();

        C7Vector qInv(boxM.getTransformation().getInverse());
        std::vector<C3Vector> triCenters;
        C3Vector avgTriCenter;
        avgTriCenter.clear();
        for (size_t i=0;i<triIndices.size();i++)
        {
            C3Vector v1(&allVertices[0]+3*allTriangles[3*size_t(triIndices[i])+0]);
            v1=qInv*v1;
            C3Vector v2(&allVertices[0]+3*allTriangles[3*size_t(triIndices[i])+1]);
            v2=qInv*v2;
            C3Vector v3(&allVertices[0]+3*allTriangles[3*size_t(triIndices[i])+2]);
            v3=qInv*v3;
            C3Vector avg=(v1+v2+v3)/double(3.0);
            triCenters.push_back(avg);
            avgTriCenter+=avg;
        }
        avgTriCenter/=double(triIndices.size());

        // Now split the triangles:
        for (size_t i=0;i<triCenters.size();i++)
        {
            C3Vector avg=triCenters[i];
            if (avg(size_t(bestAxis))<avgTriCenter(size_t(bestAxis)))
                triIndicesGroup1.push_back(triIndices[i]);
            else
                triIndicesGroup2.push_back(triIndices[i]);
        }

        // To avoid unbalanced situations:
        double q=double(triIndicesGroup1.size())/double(triIndicesGroup2.size()+1);
        if (q>1.0)
            q=1.0/q;
        if (q>double(0.25))
            break;
        // If we have a bad ratio, we try a different axis
    }

    // To avoid unbalanced situations, last chance:
    double q=double(triIndicesGroup1.size())/double(triIndicesGroup2.size()+1);
    if (q>1.0)
        q=1.0/q;
    if (q<double(0.25))
    { // we have a bad ratio. We simply equally split the triangles:
        triIndicesGroup1.assign(triIndices.begin(),triIndices.begin()+triIndices.size()/2);
        triIndicesGroup2.assign(triIndices.begin()+triIndices.size()/2,triIndices.end());
    }
}

C4X4Matrix CObbNode::getNaturalFrame(const std::vector<double>& allVertices,const std::vector<int>& allTriangles,const std::vector<int>& trianglesIndices,C3Vector& boxHs)
{
    C4X4Matrix bestM;
    C4Vector bestQ;
    bestQ.setIdentity();
    bestM.setIdentity();
    boxHs.setData(double(0.00001),double(0.00001),double(0.00001));
    if (trianglesIndices.size()!=0)
    {
        std::vector<C3Vector> usedVertices;
        std::vector<bool> usedV(allVertices.size(),false);
        for (size_t i=0;i<trianglesIndices.size();i++)
        {
            for (size_t j=0;j<3;j++)
            {
                int ind=3*allTriangles[3*trianglesIndices[i]+j];
                if (!usedV[ind])
                {
                    usedV[ind]=true;
                    C3Vector v(&allVertices[0]+ind);
                    usedVertices.push_back(v);
                }
            }
        }

        double smallestZ=DBL_MAX;
        const double* orientations[2]={q1,q2};
        int orientationsCnt[2]={91,36};
        for (size_t l=0;l<2;l++)
        {
            for (size_t i=0;i<orientationsCnt[l];i++)
            { // test all orientations and select the one with the smallest z-axis
                C4Vector q(orientations[l]+4*i);
                q=bestQ*q;
                C4Vector qInv(q.getInverse());
                double minV=DBL_MAX;
                double maxV=-DBL_MAX;
                double l;
                for (size_t j=0;j<usedVertices.size();j++)
                {
                    C3Vector v(qInv*usedVertices[j]);
                    if (v(2)<minV)
                        minV=v(2);
                    if (v(2)>maxV)
                        maxV=v(2);
                    l=maxV-minV;
                    if (l>=smallestZ)
                        break;
                }
                if (l<smallestZ)
                {
                    smallestZ=l;
                    bestQ=q;
                }
            }
        }

        double smallestVolume=DBL_MAX;
        const double* circularOrientations[2]={q3,q4};
        int circularOrientationsCnt[2]={10,10};
        for (size_t l=0;l<2;l++)
        {
            for (size_t i=0;i<circularOrientationsCnt[l];i++)
            { // test all circular orientations on top of the previous found and the one with the smallest volume
                C4Vector q(circularOrientations[l]+4*i);
                q=bestQ*q;
                C4Vector qInv(q.getInverse());
                C3Vector minV(DBL_MAX,DBL_MAX,DBL_MAX);
                C3Vector maxV(-DBL_MAX,-DBL_MAX,-DBL_MAX);
                double vol;
                C3Vector dim;
                for (size_t j=0;j<usedVertices.size();j++)
                {
                    C3Vector v(qInv*usedVertices[j]);
                    minV.keepMin(v);
                    maxV.keepMax(v);
                    dim=maxV-minV;
                    vol=dim(0)*dim(1)*dim(2);
                    if (vol>=smallestVolume)
                        break;
                }
                if (vol<smallestVolume)
                {
                    smallestVolume=vol;
                    boxHs=dim*0.5;
                    bestM.M=q.getMatrix();
                    bestM.X=bestM.M*((minV+maxV)*0.5);
                }
            }
        }
    }
    return(bestM);
}

bool CObbNode::checkCollision_obb(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const CObbNode* obb2,const CObbStruct* obbStruct2,const C4X4Matrix& shape2M,std::vector<double>* intersections,int* cachingTri1,int* cachingTri2) const
{
    bool retVal=false;
    if (leafTris==nullptr)
    {
        if ( (obb2->leafTris==nullptr)&&(boxHs(0)*boxHs(1)*boxHs(2)>=obb2->boxHs(0)*obb2->boxHs(1)*obb2->boxHs(2)) )
        { // we first explore the larger box
            CObbNode* nodes[2];
            if (obbNodes[0]->boxHs(0)*obbNodes[0]->boxHs(1)*obbNodes[0]->boxHs(2)>=obbNodes[1]->boxHs(0)*obbNodes[1]->boxHs(1)*obbNodes[1]->boxHs(2))
            { // we first explore the larger box
                nodes[0]=obbNodes[0];
                nodes[1]=obbNodes[1];
            }
            else
            {
                nodes[0]=obbNodes[1];
                nodes[1]=obbNodes[0];
            }
            for (size_t i=0;i<2;i++)
            {
                if (CCalcUtils::doCollide_box_box(shape1M*nodes[i]->boxM,nodes[i]->boxHs,shape2M*obb2->boxM,obb2->boxHs,true))
                {
                    if (nodes[i]->checkCollision_obb(obbStruct1,shape1M,obb2,obbStruct2,shape2M,intersections,cachingTri1,cachingTri2))
                    {
                        retVal=true;
                        if (intersections==nullptr)
                            break;
                    }
                }
            }
        }
        else
            retVal=obb2->checkCollision_obb(obbStruct2,shape2M,this,obbStruct1,shape1M,intersections,cachingTri2,cachingTri1);
    }
    else
    {
        for (size_t i=0;i<leafTris->size();i++)
        {
            int ind=3*leafTris[0][i];
            C3Vector p(&obbStruct1->vertices[3*obbStruct1->indices[ind+0]+0]);
            C3Vector v(&obbStruct1->vertices[3*obbStruct1->indices[ind+1]+0]);
            C3Vector w(&obbStruct1->vertices[3*obbStruct1->indices[ind+2]+0]);
            p*=shape1M;
            v*=shape1M;
            w*=shape1M;
            v-=p;
            w-=p;
            bool b=obb2->checkCollision_tri(obbStruct2,shape2M,p,v,w,ind/3,intersections,cachingTri2,cachingTri1);
            retVal=retVal||b;
            if (retVal&&(intersections==nullptr))
                break;
        }
    }
    return(retVal);
}

bool CObbNode::checkCollision_tri(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,int tri2Index,std::vector<double>* intersections,int* cachingTri1,int* cachingTri2) const
{
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];
        if (obbNodes[0]->boxHs(0)*obbNodes[0]->boxHs(1)*obbNodes[0]->boxHs(2)>=obbNodes[1]->boxHs(0)*obbNodes[1]->boxHs(1)*obbNodes[1]->boxHs(2))
        { // we first explore the larger box
            nodes[0]=obbNodes[0];
            nodes[1]=obbNodes[1];
        }
        else
        {
            nodes[0]=obbNodes[1];
            nodes[1]=obbNodes[0];
        }
        for (size_t i=0;i<2;i++)
        {
            if (CCalcUtils::doCollide_box_tri(shape1M*nodes[i]->boxM,nodes[i]->boxHs,true,p2,v2,w2))
            {
                if (nodes[i]->checkCollision_tri(obbStruct1,shape1M,p2,v2,w2,tri2Index,intersections,cachingTri1,cachingTri2))
                {
                    retVal=true;
                    if (intersections==nullptr)
                        break;
                }
            }
        }
    }
    else
    {
        for (size_t i=0;i<leafTris->size();i++)
        {
            int ind1=3*leafTris[0][i];
            C3Vector p1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+0]+0]);
            C3Vector v1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+1]+0]);
            C3Vector w1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+2]+0]);
            p1*=shape1M;
            v1*=shape1M;
            w1*=shape1M;
            v1-=p1;
            w1-=p1;
            bool b=CCalcUtils::doCollide_tri_tri(p1,v1,w1,ind1/3,p2,v2,w2,tri2Index,intersections,cachingTri1,cachingTri2);
            retVal=retVal||b;
            if (retVal&&(intersections==nullptr))
                break;
        }
    }
    return(retVal);
}

bool CObbNode::checkCollision_segp(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& segP,const C3Vector& segL,std::vector<double>* intersections,int* cachingTri1) const
{
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];
        if (obbNodes[0]->boxHs(0)*obbNodes[0]->boxHs(1)*obbNodes[0]->boxHs(2)>=obbNodes[1]->boxHs(0)*obbNodes[1]->boxHs(1)*obbNodes[1]->boxHs(2))
        { // we first explore the larger box
            nodes[0]=obbNodes[0];
            nodes[1]=obbNodes[1];
        }
        else
        {
            nodes[0]=obbNodes[1];
            nodes[1]=obbNodes[0];
        }
        for (size_t i=0;i<2;i++)
        {
            if (CCalcUtils::doCollide_box_segp(shape1M*nodes[i]->boxM,nodes[i]->boxHs,true,segP,segL))
            {
                if (nodes[i]->checkCollision_segp(obbStruct1,shape1M,segP,segL,intersections,cachingTri1))
                {
                    retVal=true;
                    if (intersections==nullptr)
                        break;
                }
            }
        }
    }
    else
    {
        C3Vector _intersect;
        C3Vector* intersect=nullptr;
        if (intersections!=nullptr)
            intersect=&_intersect;
        for (size_t i=0;i<leafTris->size();i++)
        {
            int ind1=3*leafTris[0][i];
            C3Vector p1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+0]+0]);
            C3Vector v1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+1]+0]);
            C3Vector w1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+2]+0]);
            p1*=shape1M;
            v1*=shape1M;
            w1*=shape1M;
            v1-=p1;
            w1-=p1;
            bool res=CCalcUtils::doCollide_tri_segp(p1,v1,w1,ind1/3,segP,segL,intersect,cachingTri1);
            if (res&&(intersect!=nullptr))
            {
                intersections->push_back(_intersect(0));
                intersections->push_back(_intersect(1));
                intersections->push_back(_intersect(2));
                intersections->push_back(_intersect(0));
                intersections->push_back(_intersect(1));
                intersections->push_back(_intersect(2));
            }
            retVal=retVal||res;
            if (retVal&&(intersections==nullptr))
                break;
        }
    }
    return(retVal);
}

bool CObbNode::checkCollision_segp_withTriInfo(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& segP,const C3Vector& segL,C3Vector& intersection,int& triangleIndex) const
{
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];
        if (obbNodes[0]->boxHs(0)*obbNodes[0]->boxHs(1)*obbNodes[0]->boxHs(2)>=obbNodes[1]->boxHs(0)*obbNodes[1]->boxHs(1)*obbNodes[1]->boxHs(2))
        { // we first explore the larger box
            nodes[0]=obbNodes[0];
            nodes[1]=obbNodes[1];
        }
        else
        {
            nodes[0]=obbNodes[1];
            nodes[1]=obbNodes[0];
        }
        for (size_t i=0;i<2;i++)
        {
            if (CCalcUtils::doCollide_box_segp(shape1M*nodes[i]->boxM,nodes[i]->boxHs,true,segP,segL))
            {
                if (nodes[i]->checkCollision_segp_withTriInfo(obbStruct1,shape1M,segP,segL,intersection,triangleIndex))
                {
                    retVal=true;
                    break;
                }
            }
        }
    }
    else
    {
        for (size_t i=0;i<leafTris->size();i++)
        {
            int ind1=3*leafTris[0][i];
            C3Vector p1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+0]+0]);
            C3Vector v1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+1]+0]);
            C3Vector w1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+2]+0]);
            p1*=shape1M;
            v1*=shape1M;
            w1*=shape1M;
            v1-=p1;
            w1-=p1;
            retVal=CCalcUtils::doCollide_tri_segp(p1,v1,w1,-1,segP,segL,&intersection,nullptr);
            if (retVal)
            {
                triangleIndex=leafTris[0][i];
                break;
            }
        }
    }
    return(retVal);
}

bool CObbNode::checkDistance_obb(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const CObbNode* obb2,const CObbStruct* obbStruct2,const C4X4Matrix& shape2M,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2,int* cachingTri1,int* cachingTri2) const
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        if ( (obb2->leafTris==nullptr)&&(boxHs(0)*boxHs(1)*boxHs(2)>=obb2->boxHs(0)*obb2->boxHs(1)*obb2->boxHs(2)) )
        { // we first explore the larger box
            const CObbNode* nodes[2];

            // Faster by exploring the closest sub-box first, instead of exploring the larger sub-box first
            double d[2]={dist,dist};
            bool one=CCalcUtils::isApproxDistanceSmaller_box_box_fast(shape1M*obbNodes[0]->boxM,obbNodes[0]->boxHs,shape2M*obb2->boxM,obb2->boxHs,d[0]);
            bool two=CCalcUtils::isApproxDistanceSmaller_box_box_fast(shape1M*obbNodes[1]->boxM,obbNodes[1]->boxHs,shape2M*obb2->boxM,obb2->boxHs,d[1]);
            if (one||two)
            {
                if (d[0]<=d[1])
                { // we first explore the closest box
                    nodes[0]=obbNodes[0];
                    nodes[1]=obbNodes[1];
                }
                else
                {
                    nodes[0]=obbNodes[1];
                    nodes[1]=obbNodes[0];
                }
                for (size_t i=0;i<2;i++)
                {
                    bool bb=nodes[i]->checkDistance_obb(obbStruct1,shape1M,obb2,obbStruct2,shape2M,dist,minDistSegPt1,minDistSegPt2,cachingTri1,cachingTri2);
                    retVal=retVal||bb;
                    if (dist<=d[1])
                        break;
                }
            }
        }
        else
            retVal=obb2->checkDistance_obb(obbStruct2,shape2M,this,obbStruct1,shape1M,dist,minDistSegPt2,minDistSegPt1,cachingTri2,cachingTri1);
    }
    else
    {
        for (size_t i=0;i<leafTris->size();i++)
        {
            int ind=3*leafTris[0][i];
            C3Vector p(&obbStruct1->vertices[3*obbStruct1->indices[ind+0]+0]);
            C3Vector v(&obbStruct1->vertices[3*obbStruct1->indices[ind+1]+0]);
            C3Vector w(&obbStruct1->vertices[3*obbStruct1->indices[ind+2]+0]);
            p*=shape1M;
            v*=shape1M;
            w*=shape1M;
            v-=p;
            w-=p;
            bool bb=obb2->checkDistance_tri(obbStruct2,shape2M,p,v,w,leafTris[0][i],dist,minDistSegPt2,minDistSegPt1,cachingTri2,cachingTri1);
            retVal=retVal||bb;
            if (dist==0.0)
                break;
        }
    }
    return(retVal);
}

bool CObbNode::checkDistance_tri(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,int tri2Index,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2,int* cachingTri1,int* cachingTri2) const
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        double d[2]={dist,dist};
        bool one=CCalcUtils::isApproxDistanceSmaller_box_tri_fast(shape1M*obbNodes[0]->boxM,obbNodes[0]->boxHs,p2,v2,w2,d[0]);
        bool two=CCalcUtils::isApproxDistanceSmaller_box_tri_fast(shape1M*obbNodes[1]->boxM,obbNodes[1]->boxHs,p2,v2,w2,d[1]);
        if (one||two)
        {
            if (d[0]<=d[1])
            {
                nodes[0]=obbNodes[0];
                nodes[1]=obbNodes[1];
            }
            else
            {
                nodes[0]=obbNodes[1];
                nodes[1]=obbNodes[0];
            }
            for (size_t i=0;i<2;i++)
            {
                bool bb=nodes[i]->checkDistance_tri(obbStruct1,shape1M,p2,v2,w2,tri2Index,dist,minDistSegPt1,minDistSegPt2,cachingTri1,cachingTri2);
                retVal=retVal||bb;
                if (dist<=d[1])
                    break;
            }
        }
    }
    else
    {
        for (size_t i=0;i<leafTris->size();i++)
        {
            int ind1=3*leafTris[0][i];
            C3Vector p1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+0]+0]);
            C3Vector v1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+1]+0]);
            C3Vector w1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+2]+0]);
            p1*=shape1M;
            v1*=shape1M;
            w1*=shape1M;
            v1-=p1;
            w1-=p1;
            if (CCalcUtils::getDistance_tri_tri(p1,v1,w1,p2,v2,w2,dist,minDistSegPt1,minDistSegPt2))
            {
                retVal=true;
                if (cachingTri1!=nullptr)
                    cachingTri1[0]=leafTris[0][i];
                if (cachingTri2!=nullptr)
                    cachingTri2[0]=tri2Index;
                if (dist==0.0)
                    break;
            }
        }
    }
    return(retVal);
}

bool CObbNode::checkDistance_segp(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& segP,const C3Vector& segL,double& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2,int* cachingTri1) const
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        double d[2]={dist,dist};
        bool one=CCalcUtils::isApproxDistanceSmaller_box_segp_fast(shape1M*obbNodes[0]->boxM,obbNodes[0]->boxHs,segP,segL,d[0]);
        bool two=CCalcUtils::isApproxDistanceSmaller_box_segp_fast(shape1M*obbNodes[1]->boxM,obbNodes[1]->boxHs,segP,segL,d[1]);
        if (one||two)
        {
            if (d[0]<=d[1])
            {
                nodes[0]=obbNodes[0];
                nodes[1]=obbNodes[1];
            }
            else
            {
                nodes[0]=obbNodes[1];
                nodes[1]=obbNodes[0];
            }
            for (size_t i=0;i<2;i++)
            {
                bool bb=nodes[i]->checkDistance_segp(obbStruct1,shape1M,segP,segL,dist,minDistSegPt1,minDistSegPt2,cachingTri1);
                retVal=retVal||bb;
                if (dist<=d[1])
                    break;
            }
        }
    }
    else
    {
        for (size_t i=0;i<leafTris->size();i++)
        {
            int ind1=3*leafTris[0][i];
            C3Vector p1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+0]+0]);
            C3Vector v1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+1]+0]);
            C3Vector w1(&obbStruct1->vertices[3*obbStruct1->indices[ind1+2]+0]);
            p1*=shape1M;
            v1*=shape1M;
            w1*=shape1M;
            v1-=p1;
            w1-=p1;
            if (CCalcUtils::getDistance_tri_segp(p1,v1,w1,segP,segL,dist,minDistSegPt1,minDistSegPt2))
            {
                retVal=true;
                if (cachingTri1!=nullptr)
                    cachingTri1[0]=leafTris[0][i];
                if (dist==0.0)
                    break;
            }
        }
    }
    return(retVal);
}

bool CObbNode::checkDistance_pt(const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const C3Vector& pt,double& dist,C3Vector* minDistSegPt,int* cachingTri) const
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        double d1=dist;
        bool b1=CCalcUtils::getDistance_box_pt(shapeM*obbNodes[0]->boxM,obbNodes[0]->boxHs,true,pt,d1,nullptr,nullptr);
        bool b2=CCalcUtils::getDistance_box_pt(shapeM*obbNodes[1]->boxM,obbNodes[1]->boxHs,true,pt,d1,nullptr,nullptr);
        if (b1||b2)
        {
            if (!b2)
            {
                nodes[0]=obbNodes[0];
                nodes[1]=obbNodes[1];
            }
            else
            {
                nodes[0]=obbNodes[1];
                nodes[1]=obbNodes[0];
            }
            for (size_t i=0;i<2;i++)
            {
                double d=dist;
                if ( (i==0)||CCalcUtils::getDistance_box_pt(shapeM*nodes[i]->boxM,nodes[i]->boxHs,true,pt,d,nullptr,nullptr) )
                {
                    bool bb=nodes[i]->checkDistance_pt(obbStruct,shapeM,pt,dist,minDistSegPt,cachingTri);
                    retVal=retVal||bb;
                }
            }
        }
    }
    else
    {
        for (size_t i=0;i<leafTris->size();i++)
        {
            int ind1=3*leafTris[0][i];
            C3Vector p1(&obbStruct->vertices[3*obbStruct->indices[ind1+0]+0]);
            C3Vector v1(&obbStruct->vertices[3*obbStruct->indices[ind1+1]+0]);
            C3Vector w1(&obbStruct->vertices[3*obbStruct->indices[ind1+2]+0]);
            p1*=shapeM;
            v1*=shapeM;
            w1*=shapeM;
            v1-=p1;
            w1-=p1;
            if (CCalcUtils::getDistance_tri_pt(p1,v1,w1,pt,dist,minDistSegPt))
            {
                retVal=true;
                if (cachingTri!=nullptr)
                    cachingTri[0]=ind1/3;
            }
        }
    }
    return(retVal);
}

bool CObbNode::checkSensorDistance_obb(const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,double cosAngle,bool frontDetection,bool backDetection,bool fast,double& dist,C3Vector* detectPt,C3Vector* triN)
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        double d[2]={dist,dist};
        bool one=CCalcUtils::getDistance_box_pt(shapeM*obbNodes[0]->boxM,obbNodes[0]->boxHs,true,C3Vector::zeroVector,d[0],nullptr,nullptr);
        if (one)
            one=CCalcUtils::isBoxMaybeInSensorVolume(planesIn,planesOut,shapeM*obbNodes[0]->boxM,obbNodes[0]->boxHs);
        bool two=CCalcUtils::getDistance_box_pt(shapeM*obbNodes[1]->boxM,obbNodes[1]->boxHs,true,C3Vector::zeroVector,d[1],nullptr,nullptr);
        if (two)
            two=CCalcUtils::isBoxMaybeInSensorVolume(planesIn,planesOut,shapeM*obbNodes[1]->boxM,obbNodes[1]->boxHs);
        if (one||two)
        {
            bool checkNode[2];
            if (d[0]<=d[1])
            {
                nodes[0]=obbNodes[0];
                nodes[1]=obbNodes[1];
                checkNode[0]=one;
                checkNode[1]=two;
            }
            else
            {
                nodes[0]=obbNodes[1];
                nodes[1]=obbNodes[0];
                checkNode[0]=two;
                checkNode[1]=one;
            }
            for (size_t i=0;i<2;i++)
            {
                if (checkNode[i])
                {
                    if (nodes[i]->checkSensorDistance_obb(obbStruct,shapeM,planesIn,planesOut,cosAngle,frontDetection,backDetection,fast,dist,detectPt,triN))
                    {
                        retVal=true;
                        if (fast||(dist<=d[1]))
                            break;
                    }
                }
            }
        }
    }
    else
    {
        for (size_t i=0;i<leafTris->size();i++)
        {
            int ind=3*leafTris[0][i];
            C3Vector p(&obbStruct->vertices[3*obbStruct->indices[ind+0]+0]);
            C3Vector v(&obbStruct->vertices[3*obbStruct->indices[ind+1]+0]);
            C3Vector w(&obbStruct->vertices[3*obbStruct->indices[ind+2]+0]);
            p*=shapeM;
            v*=shapeM;
            w*=shapeM;
            v-=p;
            w-=p;
            if (CCalcUtils::getSensorDistance_tri(planesIn,planesOut,cosAngle,frontDetection,backDetection,p,v,w,dist,detectPt,triN))
            {
                retVal=true;
                if (fast)
                    break;
            }
        }
    }
    return(retVal);
}

bool CObbNode::checkRaySensorDistance_obb(const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const C3Vector& raySegP,const C3Vector& raySegL,double cosAngle,bool frontDetection,bool backDetection,double forbiddenDist,bool fast,double& dist,C3Vector* detectPt,C3Vector* triN,bool* forbiddenDistTouched)
{
    if (dist==0.0)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        double d[2]={dist,dist};
        bool one=CCalcUtils::getDistance_box_pt(shapeM*obbNodes[0]->boxM,obbNodes[0]->boxHs,true,C3Vector::zeroVector,d[0],nullptr,nullptr);
        if (one)
            one=CCalcUtils::doCollide_box_segp(shapeM*obbNodes[0]->boxM,obbNodes[0]->boxHs,true,raySegP,raySegL);
        bool two=CCalcUtils::getDistance_box_pt(shapeM*obbNodes[1]->boxM,obbNodes[1]->boxHs,true,C3Vector::zeroVector,d[1],nullptr,nullptr);
        if (two)
            two=CCalcUtils::doCollide_box_segp(shapeM*obbNodes[1]->boxM,obbNodes[1]->boxHs,true,raySegP,raySegL);
        if (one||two)
        {
            bool checkNode[2];
            if (d[0]<=d[1])
            {
                nodes[0]=obbNodes[0];
                nodes[1]=obbNodes[1];
                checkNode[0]=one;
                checkNode[1]=two;
            }
            else
            {
                nodes[0]=obbNodes[1];
                nodes[1]=obbNodes[0];
                checkNode[0]=two;
                checkNode[1]=one;
            }
            for (size_t i=0;i<2;i++)
            {
                if (checkNode[i])
                {
                    if (nodes[i]->checkRaySensorDistance_obb(obbStruct,shapeM,raySegP,raySegL,cosAngle,frontDetection,backDetection,forbiddenDist,fast,dist,detectPt,triN,forbiddenDistTouched))
                    {
                        retVal=true;
                        if ( fast||((forbiddenDistTouched!=nullptr)&&forbiddenDistTouched[0])||(dist<=d[1]) )
                            break;
                    }
                }
            }
        }
    }
    else
    {
        for (size_t i=0;i<leafTris->size();i++)
        {
            int ind=3*leafTris[0][i];
            C3Vector p(&obbStruct->vertices[3*obbStruct->indices[ind+0]+0]);
            C3Vector v(&obbStruct->vertices[3*obbStruct->indices[ind+1]+0]);
            C3Vector w(&obbStruct->vertices[3*obbStruct->indices[ind+2]+0]);
            p*=shapeM;
            v*=shapeM;
            w*=shapeM;
            v-=p;
            w-=p;
            if (CCalcUtils::getRaySensorDistance_tri(raySegP,raySegL,cosAngle,frontDetection,backDetection,forbiddenDist,p,v,w,forbiddenDistTouched,dist,detectPt,triN))
            {
                retVal=true;
                if ( fast||((forbiddenDistTouched!=nullptr)&&forbiddenDistTouched[0]) )
                    break;
            }
        }
    }
    return(retVal);
}
