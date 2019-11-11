#include "calcUtils.h"
#include "obbNode.h"
#include "obbStruct.h"

const float q1[91*4]={
    1.0f,0.0f,0.0f,0.0f,
    0.99384415149689f,-0.0061558117158711f,0.078217260539532f,0.078217305243015f,
    0.96937239170074f,-0.018315907567739f,0.23272576928139f,0.076291389763355f,
    0.92103153467178f,-0.030024981126189f,0.38150382041931f,0.072486810386181f,
    0.85001176595688f,-0.040994759649038f,0.52088803052902f,0.066897414624691f,
    0.75806188583374f,-0.050955086946487f,0.64744603633881f,0.059660792350769f,
    0.94550329446793f,-0.05449678376317f,0.22699527442455f,0.22699521481991f,
    0.89835268259048f,-0.089335732161999f,0.37210986018181f,0.21567536890507f,
    0.82908165454865f,-0.1219749674201f,0.5080618262291f,0.19904483854771f,
    0.73939591646194f,-0.15161071717739f,0.63150370121002f,0.17751318216324f,
    0.92103147506714f,-0.03002499602735f,0.072486750781536f,0.38150382041931f,
    0.89835262298584f,-0.089335650205612f,0.21567541360855f,0.3721099793911f,
    0.85355335474014f,-0.14644660055637f,0.35355341434479f,0.3535535633564f,
    0.78773677349091f,-0.19995146989822f,0.48272582888603f,0.3262914121151f,
    0.70252358913422f,-0.2485329657793f,0.60001170635223f,0.29099488258362f,
    0.82908165454865f,-0.12197487056255f,0.19904492795467f,0.50806188583374f,
    0.78773683309555f,-0.19995151460171f,0.32629129290581f,0.48272570967674f,
    0.72699528932571f,-0.27300474047661f,0.44550329446793f,0.44550329446793f,
    0.6483525633812f,-0.33933570981026f,0.5537456870079f,0.39731112122536f,
    0.75806188583374f,-0.05095511302352f,0.059660773724318f,0.64744603633881f,
    0.73939591646194f,-0.15161070227623f,0.17751322686672f,0.63150376081467f,
    0.70252358913422f,-0.24853309988976f,0.29099479317665f,0.60001170635223f,
    0.64835274219513f,-0.33933570981026f,0.39731100201607f,0.55374538898468f,
    0.57821726799011f,-0.42178285121918f,0.49384421110153f,0.49384415149689f,
    0.63150358200073f,-0.17751328647137f,0.15161070227623f,0.73939609527588f,
    0.60001170635223f,-0.29099473357201f,0.24853305518627f,0.70252352952957f,
    0.55374562740326f,-0.39731097221375f,0.33933568000793f,0.64835262298584f,
    0.49384415149689f,-0.49384415149689f,0.42178285121918f,0.57821726799011f,
    0.52088797092438f,-0.066897362470627f,0.040994789451361f,0.85001170635223f,
    0.50806170701981f,-0.199044957757f,0.12197488546371f,0.82908183336258f,
    0.4827256500721f,-0.32629129290581f,0.19995157420635f,0.7877368927002f,
    0.4455032646656f,-0.44550320506096f,0.27300474047661f,0.72699534893036f,
    0.39731097221375f,-0.55374550819397f,0.33933565020561f,0.64835268259048f,
    0.37210977077484f,-0.21567541360855f,0.089335687458515f,0.89835274219513f,
    0.35355335474014f,-0.35355344414711f,0.14644664525986f,0.85355341434479f,
    0.32629126310349f,-0.48272567987442f,0.19995155930519f,0.78773683309555f,
    0.29099473357201f,-0.60001170635223f,0.24853302538395f,0.70252352952957f,
    0.23272578418255f,-0.076291278004646f,0.018315944820642f,0.96937245130539f,
    0.22699522972107f,-0.22699525952339f,0.054496757686138f,0.94550329446793f,
    0.21567545831203f,-0.37210991978645f,0.08933574706316f,0.89835262298584f,
    0.19904497265816f,-0.50806188583374f,0.12197491526604f,0.82908165454865f,
    0.17751327157021f,-0.63150376081467f,0.15161073207855f,0.73939591646194f,
    0.076291263103485f,-0.23272579908371f,0.018315916880965f,0.96937245130539f,
    0.072486750781536f,-0.38150373101234f,0.03002498857677f,0.92103159427643f,
    0.066897392272949f,-0.52088785171509f,0.040994752198458f,0.85001176595688f,
    0.059660766273737f,-0.64744603633881f,0.05095511674881f,0.75806188583374f,
    -0.078217208385468f,-0.078217223286629f,-0.006155826151371f,0.99384421110153f,
    -0.076291233301163f,-0.23272578418255f,-0.018315909430385f,0.96937245130539f,
    -0.072486698627472f,-0.38150376081467f,-0.030024975538254f,0.92103159427643f,
    -0.066897362470627f,-0.52088785171509f,-0.040994733572006f,0.85001182556152f,
    -0.059660736471415f,-0.64744603633881f,-0.050955090671778f,0.75806194543839f,
    -0.22699531912804f,-0.22699527442455f,-0.054496750235558f,0.94550323486328f,
    -0.21567545831203f,-0.37210991978645f,-0.089335672557354f,0.89835268259048f,
    -0.19904491305351f,-0.50806188583374f,-0.12197485566139f,0.82908165454865f,
    -0.17751330137253f,-0.63150376081467f,-0.15161062777042f,0.7393958568573f,
    -0.38150364160538f,-0.072486743330956f,-0.030024966225028f,0.92103159427643f,
    -0.37210974097252f,-0.21567544341087f,-0.089335672557354f,0.89835274219513f,
    -0.3535532951355f,-0.35355341434479f,-0.1464466303587f,0.85355347394943f,
    -0.32629123330116f,-0.48272570967674f,-0.19995157420635f,0.7877368927002f,
    -0.29099464416504f,-0.60001176595688f,-0.24853302538395f,0.70252358913422f,
    0.50806170701981f,0.199044957757f,0.1219748929143f,-0.82908171415329f,
    -0.48272567987442f,-0.32629126310349f,-0.19995151460171f,0.78773683309555f,
    -0.44550320506096f,-0.44550329446793f,-0.27300471067429f,0.72699534893036f,
    -0.39731097221375f,-0.55374556779861f,-0.33933565020561f,0.64835274219513f,
    0.64744591712952f,0.059660769999027f,0.050955083221197f,-0.75806200504303f,
    0.63150358200073f,0.17751330137253f,0.15161070227623f,-0.73939609527588f,
    0.60001164674759f,0.29099479317665f,0.24853308498859f,-0.70252364873886f,
    0.55374544858932f,0.39731103181839f,0.3393357694149f,-0.64835274219513f,
    -0.49384412169456f,-0.49384415149689f,-0.42178279161453f,0.57821726799011f,
    0.73939591646194f,0.15161064267159f,0.17751328647137f,-0.63150376081467f,
    0.70252352952957f,0.24853302538395f,0.29099476337433f,-0.60001170635223f,
    0.64835262298584f,0.3393357694149f,0.39731109142303f,-0.55374556779861f,
    0.57821714878082f,0.42178282141685f,0.49384421110153f,-0.49384421110153f,
    0.85001182556152f,0.040994759649038f,0.066897377371788f,-0.52088791131973f,
    0.82908165454865f,0.12197490036488f,0.19904498755932f,-0.50806188583374f,
    0.78773683309555f,0.19995154440403f,0.32629129290581f,-0.48272573947906f,
    0.72699528932571f,0.27300474047661f,0.44550332427025f,-0.4455032646656f,
    0.64835268259048f,0.33933568000793f,0.55374556779861f,-0.39731100201607f,
    0.89835262298584f,0.089335709810257f,0.21567544341087f,-0.37210991978645f,
    0.85355341434479f,0.14644664525986f,0.35355338454247f,-0.35355335474014f,
    0.78773677349091f,0.19995158910751f,0.48272570967674f,-0.32629129290581f,
    0.70252346992493f,0.24853307008743f,0.60001182556152f,-0.29099479317665f,
    0.96937245130539f,0.018315913155675f,0.076291263103485f,-0.2327257245779f,
    0.94550323486328f,0.054496750235558f,0.22699530422688f,-0.22699522972107f,
    0.89835268259048f,0.089335694909096f,0.37210991978645f,-0.21567544341087f,
    0.82908165454865f,0.12197487801313f,0.50806188583374f,-0.19904491305351f,
    0.73939591646194f,0.15161064267159f,0.63150370121002f,-0.17751325666904f,
    0.96937245130539f,0.018315905705094f,0.23272575438023f,-0.076291218400002f,
    0.92103153467178f,0.030024981126189f,0.38150382041931f,-0.072486713528633f,
    0.85001182556152f,0.040994741022587f,0.52088791131973f,-0.066897377371788f,
    0.75806188583374f,0.050955094397068f,0.64744609594345f,-0.059660740196705f
};

const float q2[36*4]={
    0.98432183265686f,-0.0054544419981539f,0.030933555215597f,0.17356246709824f,
    0.98345810174942f,-0.0090880254283547f,0.051540859043598f,0.17341016232967f,
    0.98216301202774f,-0.012717707082629f,0.072125561535358f,0.17318181693554f,
    0.98043715953827f,-0.016341743990779f,0.092678628861904f,0.17287746071815f,
    0.86559802293777f,-0.015705356374383f,0.027202559635043f,0.49975338578224f,
    0.86483854055405f,-0.02616797760129f,0.045324314385653f,0.49931478500366f,
    0.86369961500168f,-0.036619070917368f,0.063426144421101f,0.49865740537643f,
    0.86218190193176f,-0.047054179012775f,0.081500194966793f,0.4977810382843f,
    0.64247047901154f,-0.024062059819698f,0.020190495997667f,0.76566648483276f,
    0.64190673828125f,-0.040091678500175f,0.033640909940004f,0.76499456167221f,
    0.64106142520905f,-0.056103721261024f,0.047076646238565f,0.76398718357086f,
    0.63993483781815f,-0.072091154754162f,0.060491673648357f,0.76264482736588f,
    0.34185141324997f,-0.029516492038965f,0.010743148624897f,0.939228951931f,
    0.34155142307281f,-0.049179725348949f,0.017899960279465f,0.93840479850769f,
    0.34110167622566f,-0.068821407854557f,0.025048969313502f,0.93716907501221f,
    0.34050220251083f,-0.088432893157005f,0.032186944037676f,0.93552225828171f,
    -2.9802322387695e-08f,-0.031410802155733f,2.0489096641541e-08f,0.99950659275055f,
    -2.9802318834982e-08f,-0.052335970103741f,-1.8626449271864e-09f,0.99862957000732f,
    -2.9802325940409e-08f,-0.073238231241703f,1.8626453268666e-08f,0.99731451272964f,
    -1.1920928955078e-07f,-0.094108313322067f,-3.7252907425511e-09f,0.99556201696396f,
    -0.34185135364532f,-0.029516480863094f,-0.010743118822575f,0.93922901153564f,
    -0.34155142307281f,-0.04917972907424f,-0.017899954691529f,0.93840485811234f,
    -0.34110155701637f,-0.068821392953396f,-0.025048937648535f,0.93716907501221f,
    -0.34050226211548f,-0.088432945311069f,-0.032186958938837f,0.93552225828171f,
    0.64247035980225f,0.024062030017376f,0.02019046805799f,-0.76566654443741f,
    0.64190655946732f,0.040091685950756f,0.033640891313553f,-0.76499480009079f,
    0.64106148481369f,0.056103706359863f,0.047076635062695f,-0.76398718357086f,
    0.63993483781815f,0.072091184556484f,0.060491684824228f,-0.76264482736588f,
    0.86559802293777f,0.015705425292253f,0.027202542871237f,-0.49975338578224f,
    0.8648384809494f,0.026167996227741f,0.04532428830862f,-0.4993149638176f,
    0.86369961500168f,0.036619134247303f,0.063426159322262f,-0.49865740537643f,
    0.86218190193176f,0.047054175287485f,0.081500202417374f,-0.49778109788895f,
    0.98432177305222f,0.0054544052109122f,0.030933560803533f,-0.17356267571449f,
    0.98345810174942f,0.0090880719944835f,0.051540851593018f,-0.17341040074825f,
    0.98216301202774f,0.012717668898404f,0.072125554084778f,-0.17318204045296f,
    0.98043709993362f,0.016341753304005f,0.092678636312485f,-0.17287777364254f
};

const float q3[10*4]={
    0.95105654001236f,0.0f,0.0f,-0.30901706218719f,
    0.9723699092865f,0.0f,0.0f,-0.23344537615776f,
    0.98768836259842f,0.0f,0.0f,-0.1564345061779f,
    0.99691736698151f,0.0f,0.0f,-0.078459121286869f,
    0.99691736698151f,0.0f,0.0f,0.078459121286869f,
    0.98768836259842f,0.0f,0.0f,0.1564345061779f,
    0.9723699092865f,0.0f,0.0f,0.23344537615776f,
    0.95105654001236f,0.0f,0.0f,0.30901706218719f,
    0.9238795042038f,0.0f,0.0f,0.38268345594406f,
    0.89100652933121f,0.0f,0.0f,0.45399054884911f
};

const float q4[10*4]={
    0.9993908405304f,0.0f,0.0f,-0.034899495542049f,
    0.99965733289719f,0.0f,0.0f,-0.026176949962974f,
    0.9998477101326f,0.0f,0.0f,-0.017452405765653f,
    0.99996191263199f,0.0f,0.0f,-0.0087265353649855f,
    0.99996191263199f,0.0f,0.0f,0.0087265353649855f,
    0.9998477101326f,0.0f,0.0f,0.017452405765653f,
    0.99965733289719f,0.0f,0.0f,0.026176949962974f,
    0.9993908405304f,0.0f,0.0f,0.034899495542049f,
    0.99904823303223f,0.0f,0.0f,0.043619386851788f,
    0.99862951040268f,0.0f,0.0f,0.05233595892787f
};

CObbNode::CObbNode()
{
    obbNodes=nullptr;
    leafTris=nullptr;
}

CObbNode::CObbNode(const std::vector<float>& allVertices,const std::vector<int>& allTriangles,const std::vector<int>& triIndices,size_t maxTriCnt)
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

void CObbNode::scaleYourself(float f)
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
    pushData(data,&tr.X(0),sizeof(float));
    pushData(data,&tr.X(1),sizeof(float));
    pushData(data,&tr.X(2),sizeof(float));
    pushData(data,&tr.Q(0),sizeof(float));
    pushData(data,&tr.Q(1),sizeof(float));
    pushData(data,&tr.Q(2),sizeof(float));
    pushData(data,&tr.Q(3),sizeof(float));
    pushData(data,&boxHs(0),sizeof(float));
    pushData(data,&boxHs(1),sizeof(float));
    pushData(data,&boxHs(2),sizeof(float));
    if (leafTris!=nullptr)
    {
        data.push_back(0);
        unsigned short w=(unsigned short)leafTris->size();
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
    tr.X(0)=((const float*)(data+pos))[0];pos+=sizeof(float);
    tr.X(1)=((const float*)(data+pos))[0];pos+=sizeof(float);
    tr.X(2)=((const float*)(data+pos))[0];pos+=sizeof(float);
    tr.Q(0)=((const float*)(data+pos))[0];pos+=sizeof(float);
    tr.Q(1)=((const float*)(data+pos))[0];pos+=sizeof(float);
    tr.Q(2)=((const float*)(data+pos))[0];pos+=sizeof(float);
    tr.Q(3)=((const float*)(data+pos))[0];pos+=sizeof(float);
    boxM=tr.getMatrix();
    boxHs(0)=((const float*)(data+pos))[0];pos+=sizeof(float);
    boxHs(1)=((const float*)(data+pos))[0];pos+=sizeof(float);
    boxHs(2)=((const float*)(data+pos))[0];pos+=sizeof(float);
    unsigned char leafn=data[pos++];
    if (leafn==0)
    {
        size_t triCnt=((const unsigned short*)(data+pos))[0];pos+=sizeof(unsigned short);
        leafTris=new std::vector<int>;
        for (size_t i=0;i<triCnt;i++)
        {
            leafTris->push_back(((const int*)(data+pos))[0]);
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

void CObbNode::splitTriangles(const std::vector<float>& allVertices,const std::vector<int>& allTriangles,const std::vector<int>& triIndices,std::vector<int>& triIndicesGroup1,std::vector<int>& triIndicesGroup2)
{
    C3Vector hs(boxHs);

    for (size_t loop=0;loop<3;loop++)
    {
        int bestAxis=0;
        float bestAxisS=0.0f;
        for (size_t i=0;i<3;i++)
        {
            if (hs(i)>bestAxisS)
            {
                bestAxisS=hs(i);
                hs(i)=0.0f;
                bestAxis=i;
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
            C3Vector v1(&allVertices[0]+3*allTriangles[3*triIndices[i]+0]);
            v1=qInv*v1;
            C3Vector v2(&allVertices[0]+3*allTriangles[3*triIndices[i]+1]);
            v2=qInv*v2;
            C3Vector v3(&allVertices[0]+3*allTriangles[3*triIndices[i]+2]);
            v3=qInv*v3;
            C3Vector avg=(v1+v2+v3)/3.0f;
            triCenters.push_back(avg);
            avgTriCenter+=avg;
        }
        avgTriCenter/=float(triIndices.size());

        // Now split the triangles:
        for (size_t i=0;i<triCenters.size();i++)
        {
            C3Vector avg=triCenters[i];
            if (avg(bestAxis)<avgTriCenter(bestAxis))
                triIndicesGroup1.push_back(triIndices[i]);
            else
                triIndicesGroup2.push_back(triIndices[i]);
        }

        // To avoid unbalanced situations:
        float q=float(triIndicesGroup1.size())/float(triIndicesGroup2.size()+1);
        if (q>1.0f)
            q=1.0f/q;
        if (q>0.25f)
            break;
        // If we have a bad ratio, we try a different axis
    }

    // To avoid unbalanced situations, last chance:
    float q=float(triIndicesGroup1.size())/float(triIndicesGroup2.size()+1);
    if (q>1.0f)
        q=1.0f/q;
    if (q<0.25f)
    { // we have a bad ratio. We simply equally split the triangles:
        triIndicesGroup1.assign(triIndices.begin(),triIndices.begin()+triIndices.size()/2);
        triIndicesGroup2.assign(triIndices.begin()+triIndices.size()/2,triIndices.end());
    }
}

C4X4Matrix CObbNode::getNaturalFrame(const std::vector<float>& allVertices,const std::vector<int>& allTriangles,const std::vector<int>& trianglesIndices,C3Vector& boxHs)
{
    C4X4Matrix bestM;
    C4Vector bestQ;
    bestQ.setIdentity();
    bestM.setIdentity();
    boxHs.set(0.00001f,0.00001f,0.00001f);
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

        float smallestZ=SIM_MAX_FLOAT;
        const float* orientations[2]={q1,q2};
        int orientationsCnt[2]={91,36};
        for (size_t l=0;l<2;l++)
        {
            for (size_t i=0;i<orientationsCnt[l];i++)
            { // test all orientations and select the one with the smallest z-axis
                C4Vector q(orientations[l]+4*i);
                q=bestQ*q;
                C4Vector qInv(q.getInverse());
                float minV=SIM_MAX_FLOAT;
                float maxV=-SIM_MAX_FLOAT;
                float l;
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

        float smallestVolume=SIM_MAX_FLOAT;
        const float* circularOrientations[2]={q3,q4};
        int circularOrientationsCnt[2]={10,10};
        for (size_t l=0;l<2;l++)
        {
            for (size_t i=0;i<circularOrientationsCnt[l];i++)
            { // test all circular orientations on top of the previous found and the one with the smallest volume
                C4Vector q(circularOrientations[l]+4*i);
                q=bestQ*q;
                C4Vector qInv(q.getInverse());
                C3Vector minV(SIM_MAX_FLOAT,SIM_MAX_FLOAT,SIM_MAX_FLOAT);
                C3Vector maxV(-SIM_MAX_FLOAT,-SIM_MAX_FLOAT,-SIM_MAX_FLOAT);
                float vol;
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
                    boxHs=dim*0.5f;
                    bestM.M=q.getMatrix();
                    bestM.X=bestM.M*((minV+maxV)*0.5f);
                }
            }
        }
    }
    return(bestM);
}

bool CObbNode::checkCollision_obb(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const CObbNode* obb2,const CObbStruct* obbStruct2,const C4X4Matrix& shape2M,std::vector<float>* intersections,int* cachingTri1,int* cachingTri2) const
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

bool CObbNode::checkCollision_tri(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,int tri2Index,std::vector<float>* intersections,int* cachingTri1,int* cachingTri2) const
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

bool CObbNode::checkCollision_segp(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& segP,const C3Vector& segL,std::vector<float>* intersections,int* cachingTri1) const
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
            retVal=retVal||CCalcUtils::doCollide_tri_segp(p1,v1,w1,ind1,segP,segL,intersections,cachingTri1);
            if (retVal&&(intersections==nullptr))
                break;
        }
    }
    return(retVal);
}

bool CObbNode::checkDistance_obb(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const CObbNode* obb2,const CObbStruct* obbStruct2,const C4X4Matrix& shape2M,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2,int* cachingTri1,int* cachingTri2) const
{
    if (dist==0.0f)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        if ( (obb2->leafTris==nullptr)&&(boxHs(0)*boxHs(1)*boxHs(2)>=obb2->boxHs(0)*obb2->boxHs(1)*obb2->boxHs(2)) )
        { // we first explore the larger box
            const CObbNode* nodes[2];

            // Faster by exploring the closest sub-box first, instead of exploring the larger sub-box first
            float d[2]={dist,dist};
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
            if (dist==0.0f)
                break;
        }
    }
    return(retVal);
}

bool CObbNode::checkDistance_tri(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& p2,const C3Vector& v2,const C3Vector& w2,int tri2Index,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2,int* cachingTri1,int* cachingTri2) const
{
    if (dist==0.0f)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        float d[2]={dist,dist};
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
                if (dist==0.0f)
                    break;
            }
        }
    }
    return(retVal);
}

bool CObbNode::checkDistance_segp(const CObbStruct* obbStruct1,const C4X4Matrix& shape1M,const C3Vector& segP,const C3Vector& segL,float& dist,C3Vector* minDistSegPt1,C3Vector* minDistSegPt2,int* cachingTri1) const
{
    if (dist==0.0f)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        float d[2]={dist,dist};
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
                if (dist==0.0f)
                    break;
            }
        }
    }
    return(retVal);
}

bool CObbNode::checkDistance_pt(const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const C3Vector& pt,float& dist,C3Vector* minDistSegPt,int* cachingTri) const
{
    if (dist==0.0f)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        float d1=dist;
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
                float d=dist;
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
                    cachingTri[0]=ind1;
            }
        }
    }
    return(retVal);
}

bool CObbNode::checkSensorDistance_obb(const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const CVolumePlanes& planesIn,const CVolumePlanes& planesOut,float cosAngle,bool frontDetection,bool backDetection,bool fast,float& dist,C3Vector* detectPt,C3Vector* triN)
{
    if (dist==0.0f)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        float d[2]={dist,dist};
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

bool CObbNode::checkRaySensorDistance_obb(const CObbStruct* obbStruct,const C4X4Matrix& shapeM,const C3Vector& raySegP,const C3Vector& raySegL,float cosAngle,bool frontDetection,bool backDetection,float forbiddenDist,bool fast,float& dist,C3Vector* detectPt,C3Vector* triN,bool* forbiddenDistTouched)
{
    if (dist==0.0f)
        return(false);
    bool retVal=false;
    if (leafTris==nullptr)
    {
        CObbNode* nodes[2];

        // Faster by exploring the closest box first, instead of exploring the larger box first
        float d[2]={dist,dist};
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
