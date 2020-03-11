#include "calcUtils.h"
#include "obbNode.h"
#include "obbStruct.h"

const simReal q1[91*4]={
    simReal(1.0),simReal(0.0),simReal(0.0),simReal(0.0),
    simReal(0.99384415149689),simReal(-0.0061558117158711),simReal(0.078217260539532),simReal(0.078217305243015),
    simReal(0.96937239170074),simReal(-0.018315907567739),simReal(0.23272576928139),simReal(0.076291389763355),
    simReal(0.92103153467178),simReal(-0.030024981126189),simReal(0.38150382041931),simReal(0.072486810386181),
    simReal(0.85001176595688),simReal(-0.040994759649038),simReal(0.52088803052902),simReal(0.066897414624691),
    simReal(0.75806188583374),simReal(-0.050955086946487),simReal(0.64744603633881),simReal(0.059660792350769),
    simReal(0.94550329446793),simReal(-0.05449678376317),simReal(0.22699527442455),simReal(0.22699521481991),
    simReal(0.89835268259048),simReal(-0.089335732161999),simReal(0.37210986018181),simReal(0.21567536890507),
    simReal(0.82908165454865),simReal(-0.1219749674201),simReal(0.5080618262291),simReal(0.19904483854771),
    simReal(0.73939591646194),simReal(-0.15161071717739),simReal(0.63150370121002),simReal(0.17751318216324),
    simReal(0.92103147506714),simReal(-0.03002499602735),simReal(0.072486750781536),simReal(0.38150382041931),
    simReal(0.89835262298584),simReal(-0.089335650205612),simReal(0.21567541360855),simReal(0.3721099793911),
    simReal(0.85355335474014),simReal(-0.14644660055637),simReal(0.35355341434479),simReal(0.3535535633564),
    simReal(0.78773677349091),simReal(-0.19995146989822),simReal(0.48272582888603),simReal(0.3262914121151),
    simReal(0.70252358913422),simReal(-0.2485329657793),simReal(0.60001170635223),simReal(0.29099488258362),
    simReal(0.82908165454865),simReal(-0.12197487056255),simReal(0.19904492795467),simReal(0.50806188583374),
    simReal(0.78773683309555),simReal(-0.19995151460171),simReal(0.32629129290581),simReal(0.48272570967674),
    simReal(0.72699528932571),simReal(-0.27300474047661),simReal(0.44550329446793),simReal(0.44550329446793),
    simReal(0.6483525633812),simReal(-0.33933570981026),simReal(0.5537456870079),simReal(0.39731112122536),
    simReal(0.75806188583374),simReal(-0.05095511302352),simReal(0.059660773724318),simReal(0.64744603633881),
    simReal(0.73939591646194),simReal(-0.15161070227623),simReal(0.17751322686672),simReal(0.63150376081467),
    simReal(0.70252358913422),simReal(-0.24853309988976),simReal(0.29099479317665),simReal(0.60001170635223),
    simReal(0.64835274219513),simReal(-0.33933570981026),simReal(0.39731100201607),simReal(0.55374538898468),
    simReal(0.57821726799011),simReal(-0.42178285121918),simReal(0.49384421110153),simReal(0.49384415149689),
    simReal(0.63150358200073),simReal(-0.17751328647137),simReal(0.15161070227623),simReal(0.73939609527588),
    simReal(0.60001170635223),simReal(-0.29099473357201),simReal(0.24853305518627),simReal(0.70252352952957),
    simReal(0.55374562740326),simReal(-0.39731097221375),simReal(0.33933568000793),simReal(0.64835262298584),
    simReal(0.49384415149689),simReal(-0.49384415149689),simReal(0.42178285121918),simReal(0.57821726799011),
    simReal(0.52088797092438),simReal(-0.066897362470627),simReal(0.040994789451361),simReal(0.85001170635223),
    simReal(0.50806170701981),simReal(-0.199044957757),simReal(0.12197488546371),simReal(0.82908183336258),
    simReal(0.4827256500721),simReal(-0.32629129290581),simReal(0.19995157420635),simReal(0.7877368927002),
    simReal(0.4455032646656),simReal(-0.44550320506096),simReal(0.27300474047661),simReal(0.72699534893036),
    simReal(0.39731097221375),simReal(-0.55374550819397),simReal(0.33933565020561),simReal(0.64835268259048),
    simReal(0.37210977077484),simReal(-0.21567541360855),simReal(0.089335687458515),simReal(0.89835274219513),
    simReal(0.35355335474014),simReal(-0.35355344414711),simReal(0.14644664525986),simReal(0.85355341434479),
    simReal(0.32629126310349),simReal(-0.48272567987442),simReal(0.19995155930519),simReal(0.78773683309555),
    simReal(0.29099473357201),simReal(-0.60001170635223),simReal(0.24853302538395),simReal(0.70252352952957),
    simReal(0.23272578418255),simReal(-0.076291278004646),simReal(0.018315944820642),simReal(0.96937245130539),
    simReal(0.22699522972107),simReal(-0.22699525952339),simReal(0.054496757686138),simReal(0.94550329446793),
    simReal(0.21567545831203),simReal(-0.37210991978645),simReal(0.08933574706316),simReal(0.89835262298584),
    simReal(0.19904497265816),simReal(-0.50806188583374),simReal(0.12197491526604),simReal(0.82908165454865),
    simReal(0.17751327157021),simReal(-0.63150376081467),simReal(0.15161073207855),simReal(0.73939591646194),
    simReal(0.076291263103485),simReal(-0.23272579908371),simReal(0.018315916880965),simReal(0.96937245130539),
    simReal(0.072486750781536),simReal(-0.38150373101234),simReal(0.03002498857677),simReal(0.92103159427643),
    simReal(0.066897392272949),simReal(-0.52088785171509),simReal(0.040994752198458),simReal(0.85001176595688),
    simReal(0.059660766273737),simReal(-0.64744603633881),simReal(0.05095511674881),simReal(0.75806188583374),
    simReal(-0.078217208385468),simReal(-0.078217223286629),simReal(-0.006155826151371),simReal(0.99384421110153),
    simReal(-0.076291233301163),simReal(-0.23272578418255),simReal(-0.018315909430385),simReal(0.96937245130539),
    simReal(-0.072486698627472),simReal(-0.38150376081467),simReal(-0.030024975538254),simReal(0.92103159427643),
    simReal(-0.066897362470627),simReal(-0.52088785171509),simReal(-0.040994733572006),simReal(0.85001182556152),
    simReal(-0.059660736471415),simReal(-0.64744603633881),simReal(-0.050955090671778),simReal(0.75806194543839),
    simReal(-0.22699531912804),simReal(-0.22699527442455),simReal(-0.054496750235558),simReal(0.94550323486328),
    simReal(-0.21567545831203),simReal(-0.37210991978645),simReal(-0.089335672557354),simReal(0.89835268259048),
    simReal(-0.19904491305351),simReal(-0.50806188583374),simReal(-0.12197485566139),simReal(0.82908165454865),
    simReal(-0.17751330137253),simReal(-0.63150376081467),simReal(-0.15161062777042),simReal(0.7393958568573),
    simReal(-0.38150364160538),simReal(-0.072486743330956),simReal(-0.030024966225028),simReal(0.92103159427643),
    simReal(-0.37210974097252),simReal(-0.21567544341087),simReal(-0.089335672557354),simReal(0.89835274219513),
    simReal(-0.3535532951355),simReal(-0.35355341434479),simReal(-0.1464466303587),simReal(0.85355347394943),
    simReal(-0.32629123330116),simReal(-0.48272570967674),simReal(-0.19995157420635),simReal(0.7877368927002),
    simReal(-0.29099464416504),simReal(-0.60001176595688),simReal(-0.24853302538395),simReal(0.70252358913422),
    simReal(0.50806170701981),simReal(0.199044957757),simReal(0.1219748929143),simReal(-0.82908171415329),
    simReal(-0.48272567987442),simReal(-0.32629126310349),simReal(-0.19995151460171),simReal(0.78773683309555),
    simReal(-0.44550320506096),simReal(-0.44550329446793),simReal(-0.27300471067429),simReal(0.72699534893036),
    simReal(-0.39731097221375),simReal(-0.55374556779861),simReal(-0.33933565020561),simReal(0.64835274219513),
    simReal(0.64744591712952),simReal(0.059660769999027),simReal(0.050955083221197),simReal(-0.75806200504303),
    simReal(0.63150358200073),simReal(0.17751330137253),simReal(0.15161070227623),simReal(-0.73939609527588),
    simReal(0.60001164674759),simReal(0.29099479317665),simReal(0.24853308498859),simReal(-0.70252364873886),
    simReal(0.55374544858932),simReal(0.39731103181839),simReal(0.3393357694149),simReal(-0.64835274219513),
    simReal(-0.49384412169456),simReal(-0.49384415149689),simReal(-0.42178279161453),simReal(0.57821726799011),
    simReal(0.73939591646194),simReal(0.15161064267159),simReal(0.17751328647137),simReal(-0.63150376081467),
    simReal(0.70252352952957),simReal(0.24853302538395),simReal(0.29099476337433),simReal(-0.60001170635223),
    simReal(0.64835262298584),simReal(0.3393357694149),simReal(0.39731109142303),simReal(-0.55374556779861),
    simReal(0.57821714878082),simReal(0.42178282141685),simReal(0.49384421110153),simReal(-0.49384421110153),
    simReal(0.85001182556152),simReal(0.040994759649038),simReal(0.066897377371788),simReal(-0.52088791131973),
    simReal(0.82908165454865),simReal(0.12197490036488),simReal(0.19904498755932),simReal(-0.50806188583374),
    simReal(0.78773683309555),simReal(0.19995154440403),simReal(0.32629129290581),simReal(-0.48272573947906),
    simReal(0.72699528932571),simReal(0.27300474047661),simReal(0.44550332427025),simReal(-0.4455032646656),
    simReal(0.64835268259048),simReal(0.33933568000793),simReal(0.55374556779861),simReal(-0.39731100201607),
    simReal(0.89835262298584),simReal(0.089335709810257),simReal(0.21567544341087),simReal(-0.37210991978645),
    simReal(0.85355341434479),simReal(0.14644664525986),simReal(0.35355338454247),simReal(-0.35355335474014),
    simReal(0.78773677349091),simReal(0.19995158910751),simReal(0.48272570967674),simReal(-0.32629129290581),
    simReal(0.70252346992493),simReal(0.24853307008743),simReal(0.60001182556152),simReal(-0.29099479317665),
    simReal(0.96937245130539),simReal(0.018315913155675),simReal(0.076291263103485),simReal(-0.2327257245779),
    simReal(0.94550323486328),simReal(0.054496750235558),simReal(0.22699530422688),simReal(-0.22699522972107),
    simReal(0.89835268259048),simReal(0.089335694909096),simReal(0.37210991978645),simReal(-0.21567544341087),
    simReal(0.82908165454865),simReal(0.12197487801313),simReal(0.50806188583374),simReal(-0.19904491305351),
    simReal(0.73939591646194),simReal(0.15161064267159),simReal(0.63150370121002),simReal(-0.17751325666904),
    simReal(0.96937245130539),simReal(0.018315905705094),simReal(0.23272575438023),simReal(-0.076291218400002),
    simReal(0.92103153467178),simReal(0.030024981126189),simReal(0.38150382041931),simReal(-0.072486713528633),
    simReal(0.85001182556152),simReal(0.040994741022587),simReal(0.52088791131973),simReal(-0.066897377371788),
    simReal(0.75806188583374),simReal(0.050955094397068),simReal(0.64744609594345),simReal(-0.059660740196705)
};

const simReal q2[36*4]={
    simReal(0.98432183265686),simReal(-0.0054544419981539),simReal(0.030933555215597),simReal(0.17356246709824),
    simReal(0.98345810174942),simReal(-0.0090880254283547),simReal(0.051540859043598),simReal(0.17341016232967),
    simReal(0.98216301202774),simReal(-0.012717707082629),simReal(0.072125561535358),simReal(0.17318181693554),
    simReal(0.98043715953827),simReal(-0.016341743990779),simReal(0.092678628861904),simReal(0.17287746071815),
    simReal(0.86559802293777),simReal(-0.015705356374383),simReal(0.027202559635043),simReal(0.49975338578224),
    simReal(0.86483854055405),simReal(-0.02616797760129),simReal(0.045324314385653),simReal(0.49931478500366),
    simReal(0.86369961500168),simReal(-0.036619070917368),simReal(0.063426144421101),simReal(0.49865740537643),
    simReal(0.86218190193176),simReal(-0.047054179012775),simReal(0.081500194966793),simReal(0.4977810382843),
    simReal(0.64247047901154),simReal(-0.024062059819698),simReal(0.020190495997667),simReal(0.76566648483276),
    simReal(0.64190673828125),simReal(-0.040091678500175),simReal(0.033640909940004),simReal(0.76499456167221),
    simReal(0.64106142520905),simReal(-0.056103721261024),simReal(0.047076646238565),simReal(0.76398718357086),
    simReal(0.63993483781815),simReal(-0.072091154754162),simReal(0.060491673648357),simReal(0.76264482736588),
    simReal(0.34185141324997),simReal(-0.029516492038965),simReal(0.010743148624897),simReal(0.939228951931),
    simReal(0.34155142307281),simReal(-0.049179725348949),simReal(0.017899960279465),simReal(0.93840479850769),
    simReal(0.34110167622566),simReal(-0.068821407854557),simReal(0.025048969313502),simReal(0.93716907501221),
    simReal(0.34050220251083),simReal(-0.088432893157005),simReal(0.032186944037676),simReal(0.93552225828171),
    simReal(-2.9802322387695e-08),simReal(-0.031410802155733),simReal(2.0489096641541e-08),simReal(0.99950659275055),
    simReal(-2.9802318834982e-08),simReal(-0.052335970103741),simReal(-1.8626449271864e-09),simReal(0.99862957000732),
    simReal(-2.9802325940409e-08),simReal(-0.073238231241703),simReal(1.8626453268666e-08),simReal(0.99731451272964),
    simReal(-1.1920928955078e-07),simReal(-0.094108313322067),simReal(-3.7252907425511e-09),simReal(0.99556201696396),
    simReal(-0.34185135364532),simReal(-0.029516480863094),simReal(-0.010743118822575),simReal(0.93922901153564),
    simReal(-0.34155142307281),simReal(-0.04917972907424),simReal(-0.017899954691529),simReal(0.93840485811234),
    simReal(-0.34110155701637),simReal(-0.068821392953396),simReal(-0.025048937648535),simReal(0.93716907501221),
    simReal(-0.34050226211548),simReal(-0.088432945311069),simReal(-0.032186958938837),simReal(0.93552225828171),
    simReal(0.64247035980225),simReal(0.024062030017376),simReal(0.02019046805799),simReal(-0.76566654443741),
    simReal(0.64190655946732),simReal(0.040091685950756),simReal(0.033640891313553),simReal(-0.76499480009079),
    simReal(0.64106148481369),simReal(0.056103706359863),simReal(0.047076635062695),simReal(-0.76398718357086),
    simReal(0.63993483781815),simReal(0.072091184556484),simReal(0.060491684824228),simReal(-0.76264482736588),
    simReal(0.86559802293777),simReal(0.015705425292253),simReal(0.027202542871237),simReal(-0.49975338578224),
    simReal(0.8648384809494),simReal(0.026167996227741),simReal(0.04532428830862),simReal(-0.4993149638176),
    simReal(0.86369961500168),simReal(0.036619134247303),simReal(0.063426159322262),simReal(-0.49865740537643),
    simReal(0.86218190193176),simReal(0.047054175287485),simReal(0.081500202417374),simReal(-0.49778109788895),
    simReal(0.98432177305222),simReal(0.0054544052109122),simReal(0.030933560803533),simReal(-0.17356267571449),
    simReal(0.98345810174942),simReal(0.0090880719944835),simReal(0.051540851593018),simReal(-0.17341040074825),
    simReal(0.98216301202774),simReal(0.012717668898404),simReal(0.072125554084778),simReal(-0.17318204045296),
    simReal(0.98043709993362),simReal(0.016341753304005),simReal(0.092678636312485),simReal(-0.17287777364254)
};

const simReal q3[10*4]={
    simReal(0.95105654001236),simReal(0.0),simReal(0.0),simReal(-0.30901706218719),
    simReal(0.9723699092865),simReal(0.0),simReal(0.0),simReal(-0.23344537615776),
    simReal(0.98768836259842),simReal(0.0),simReal(0.0),simReal(-0.1564345061779),
    simReal(0.99691736698151),simReal(0.0),simReal(0.0),simReal(-0.078459121286869),
    simReal(0.99691736698151),simReal(0.0),simReal(0.0),simReal(0.078459121286869),
    simReal(0.98768836259842),simReal(0.0),simReal(0.0),simReal(0.1564345061779),
    simReal(0.9723699092865),simReal(0.0),simReal(0.0),simReal(0.23344537615776),
    simReal(0.95105654001236),simReal(0.0),simReal(0.0),simReal(0.30901706218719),
    simReal(0.9238795042038),simReal(0.0),simReal(0.0),simReal(0.38268345594406),
    simReal(0.89100652933121),simReal(0.0),simReal(0.0),simReal(0.45399054884911)
};

const simReal q4[10*4]={
    simReal(0.9993908405304),simReal(0.0),simReal(0.0),simReal(-0.034899495542049),
    simReal(0.99965733289719),simReal(0.0),simReal(0.0),simReal(-0.026176949962974),
    simReal(0.9998477101326),simReal(0.0),simReal(0.0),simReal(-0.017452405765653),
    simReal(0.99996191263199),simReal(0.0),simReal(0.0),simReal(-0.0087265353649855),
    simReal(0.99996191263199),simReal(0.0),simReal(0.0),simReal(0.0087265353649855),
    simReal(0.9998477101326),simReal(0.0),simReal(0.0),simReal(0.017452405765653),
    simReal(0.99965733289719),simReal(0.0),simReal(0.0),simReal(0.026176949962974),
    simReal(0.9993908405304),simReal(0.0),simReal(0.0),simReal(0.034899495542049),
    simReal(0.99904823303223),simReal(0.0),simReal(0.0),simReal(0.043619386851788),
    simReal(0.99862951040268),simReal(0.0),simReal(0.0),simReal(0.05233595892787)
};

CObbNode::CObbNode()
{
    obbNodes=nullptr;
    leafTris=nullptr;
}

CObbNode::CObbNode(const std::vector<simReal>& allVertices,const std::vector<int>& allTriangles,const std::vector<int>& triIndices,size_t maxTriCnt)
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

void CObbNode::scaleYourself(simReal f)
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
    pushData(data,&tr.X(0),sizeof(simReal));
    pushData(data,&tr.X(1),sizeof(simReal));
    pushData(data,&tr.X(2),sizeof(simReal));
    pushData(data,&tr.Q(0),sizeof(simReal));
    pushData(data,&tr.Q(1),sizeof(simReal));
    pushData(data,&tr.Q(2),sizeof(simReal));
    pushData(data,&tr.Q(3),sizeof(simReal));
    pushData(data,&boxHs(0),sizeof(simReal));
    pushData(data,&boxHs(1),sizeof(simReal));
    pushData(data,&boxHs(2),sizeof(simReal));
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
    tr.X(0)=(reinterpret_cast<const simReal*>(data+pos))[0];pos+=sizeof(simReal);
    tr.X(1)=(reinterpret_cast<const simReal*>(data+pos))[0];pos+=sizeof(simReal);
    tr.X(2)=(reinterpret_cast<const simReal*>(data+pos))[0];pos+=sizeof(simReal);
    tr.Q(0)=(reinterpret_cast<const simReal*>(data+pos))[0];pos+=sizeof(simReal);
    tr.Q(1)=(reinterpret_cast<const simReal*>(data+pos))[0];pos+=sizeof(simReal);
    tr.Q(2)=(reinterpret_cast<const simReal*>(data+pos))[0];pos+=sizeof(simReal);
    tr.Q(3)=(reinterpret_cast<const simReal*>(data+pos))[0];pos+=sizeof(simReal);
    boxM=tr.getMatrix();
    boxHs(0)=(reinterpret_cast<const simReal*>(data+pos))[0];pos+=sizeof(simReal);
    boxHs(1)=(reinterpret_cast<const simReal*>(data+pos))[0];pos+=sizeof(simReal);
    boxHs(2)=(reinterpret_cast<const simReal*>(data+pos))[0];pos+=sizeof(simReal);
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

void CObbNode::splitTriangles(const std::vector<simReal>& allVertices,const std::vector<int>& allTriangles,const std::vector<int>& triIndices,std::vector<int>& triIndicesGroup1,std::vector<int>& triIndicesGroup2)
{
    C3Vector hs(boxHs);

    for (size_t loop=0;loop<3;loop++)
    {
        int bestAxis=0;
        simReal bestAxisS=simZero;
        for (size_t i=0;i<3;i++)
        {
            if (hs(i)>bestAxisS)
            {
                bestAxisS=hs(i);
                hs(i)=simZero;
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
            C3Vector avg=(v1+v2+v3)/simReal(3.0);
            triCenters.push_back(avg);
            avgTriCenter+=avg;
        }
        avgTriCenter/=simReal(triIndices.size());

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
        simReal q=simReal(triIndicesGroup1.size())/simReal(triIndicesGroup2.size()+1);
        if (q>simOne)
            q=simOne/q;
        if (q>simReal(0.25))
            break;
        // If we have a bad ratio, we try a different axis
    }

    // To avoid unbalanced situations, last chance:
    simReal q=simReal(triIndicesGroup1.size())/simReal(triIndicesGroup2.size()+1);
    if (q>simOne)
        q=simOne/q;
    if (q<simReal(0.25))
    { // we have a bad ratio. We simply equally split the triangles:
        triIndicesGroup1.assign(triIndices.begin(),triIndices.begin()+triIndices.size()/2);
        triIndicesGroup2.assign(triIndices.begin()+triIndices.size()/2,triIndices.end());
    }
}

C4X4Matrix CObbNode::getNaturalFrame(const std::vector<simReal>& allVertices,const std::vector<int>& allTriangles,const std::vector<int>& trianglesIndices,C3Vector& boxHs)
{
    C4X4Matrix bestM;
    C4Vector bestQ;
    bestQ.setIdentity();
    bestM.setIdentity();
    boxHs.set(simReal(0.00001),simReal(0.00001),simReal(0.00001));
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

        simReal smallestZ=SIM_MAX_REAL;
        const simReal* orientations[2]={q1,q2};
        int orientationsCnt[2]={91,36};
        for (size_t l=0;l<2;l++)
        {
            for (size_t i=0;i<orientationsCnt[l];i++)
            { // test all orientations and select the one with the smallest z-axis
                C4Vector q(orientations[l]+4*i);
                q=bestQ*q;
                C4Vector qInv(q.getInverse());
                simReal minV=SIM_MAX_REAL;
                simReal maxV=-SIM_MAX_REAL;
                simReal l;
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

        simReal smallestVolume=SIM_MAX_REAL;
        const simReal* circularOrientations[2]={q3,q4};
        int circularOrientationsCnt[2]={10,10};
        for (size_t l=0;l<2;l++)
        {
            for (size_t i=0;i<circularOrientationsCnt[l];i++)
            { // test all circular orientations on top of the previous found and the one with the smallest volume
                C4Vector q(circularOrientations[l]+4*i);
                q=bestQ*q;
                C4Vector qInv(q.getInverse());
                C3Vector minV(SIM_MAX_REAL,SIM_MAX_REAL,SIM_MAX_REAL);
                C3Vector maxV(-SIM_MAX_REAL,-SIM_MAX_REAL,-SIM_MAX_REAL);
                simReal vol;
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
                    boxHs=dim*simHalf;
                    bestM.M=q.getMatrix();
                    bestM.X=bestM.M*((minV+maxV)*simHalf);
                }
            }
        }
    }
    return(bestM);
}

bool CObbNode::checkCollision_obb(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const CObbNode* obb2,const CObbStruct* obbStruct2,const C4X4Matrix& shape2M,std::vector<simReal>* intersections,int* cachingTri1,int* cachingTri2) const
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

bool CObbNode::checkCollision_tri(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,int tri2Index,std::vector<simReal>* intersections,int* cachingTri1,int* cachingTri2) const
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

bool CObbNode::checkCollision_segp(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& segP,const C3Vector& segL,std::vector<simReal>* intersections,int* cachingTri1) const
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

bool CObbNode::checkDistance_obb(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const CObbNode* obb2,const CObbStruct* obbStruct2,const C4X4Matrix& shape2M,simReal& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2,int* cachingTri1,int* cachingTri2) const
{
    if (dist==simZero)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        if ( (obb2->leafTris==nullptr)&&(boxHs(0)*boxHs(1)*boxHs(2)>=obb2->boxHs(0)*obb2->boxHs(1)*obb2->boxHs(2)) )
        { // we first explore the larger box
            const CObbNode* nodes[2];

            // Faster by exploring the closest sub-box first, instead of exploring the larger sub-box first
            simReal d[2]={dist,dist};
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
            if (dist==simZero)
                break;
        }
    }
    return(retVal);
}

bool CObbNode::checkDistance_tri(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,int tri2Index,simReal& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2,int* cachingTri1,int* cachingTri2) const
{
    if (dist==simZero)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        simReal d[2]={dist,dist};
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
                if (dist==simZero)
                    break;
            }
        }
    }
    return(retVal);
}

bool CObbNode::checkDistance_segp(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& segP,const C3Vector& segL,simReal& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2,int* cachingTri1) const
{
    if (dist==simZero)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        simReal d[2]={dist,dist};
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
                if (dist==simZero)
                    break;
            }
        }
    }
    return(retVal);
}

bool CObbNode::checkDistance_pt(const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const C3Vector& pt,simReal& dist,C3Vector* minDistSegPt,int* cachingTri) const
{
    if (dist==simZero)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        simReal d1=dist;
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
                simReal d=dist;
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

bool CObbNode::checkSensorDistance_obb(const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,simReal cosAngle,bool frontDetection,bool backDetection,bool fast,simReal& dist,C3Vector* detectPt,C3Vector* triN)
{
    if (dist==simZero)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        simReal d[2]={dist,dist};
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

bool CObbNode::checkRaySensorDistance_obb(const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const C3Vector& raySegP,const C3Vector& raySegL,simReal cosAngle,bool frontDetection,bool backDetection,simReal forbiddenDist,bool fast,simReal& dist,C3Vector* detectPt,C3Vector* triN,bool* forbiddenDistTouched)
{
    if (dist==simZero)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        simReal d[2]={dist,dist};
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
