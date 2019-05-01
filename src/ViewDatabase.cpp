//
//  ViewDatabase.cpp
//  linfslam
//
//  Created by Alvaro Parra on 16/4/19.
//  Copyright © 2019 Alvaro Parra. All rights reserved.
//  addapted from ORBSLAM2 KeyViewDatabase

#include "ViewDatabase.hpp"

//#include "KeyFrame.h"
#include "BowVector.h"

#include<mutex>

namespace linf
{
    
    void ViewDatabase::add(View *view)
    {
        const auto &bow = view->frame().bow();
        for (const auto &w: bow)
        {
            const auto &id = w.first;
            m_inverted_file[id].push_back(view);
        }
    }
    
    
    void ViewDatabase::erase(View *view)
    {
        // Erase elements in the Inverse File for the entry
        const auto &bow = view->frame().bow();
        for (auto &w: bow)
        {
            const auto &w_id = w.first;
            
            auto &f_list = m_inverted_file[w_id];
            
            for (auto it = f_list.begin(); it != f_list.end(); it++)
            {
                if(*it == view) 
                {
                    f_list.erase(it);
                    break;
                }
            }
        }
    }
    
    
    std::list<View*> ViewDatabase::findViewsSharingWords(View &view, std::map<View*, int> &num_of_shared_words)
    {
        std::list<View *> views_sharing_words;
        
        std::map<View*, View*> cached_loops;
        
        for (auto &w: view.frame().bow())
        {
            const auto &word_id = w.first;
            for (const auto &view_sharing_a_word: m_inverted_file[word_id])
            {
                //          if(pKFi->mnLoopQuery!=pKF->mnId)
                if (cached_loops[view_sharing_a_word] != &view) //if vi is not in the map it crates a null one...
                {
                    num_of_shared_words[view_sharing_a_word] = 0; //pKFi->mnLoopWords=0;
                    
                    if( !view_sharing_a_word->isConnectedTo(view) )
                    {
                        cached_loops[view_sharing_a_word] = &view;
                        views_sharing_words.push_back(view_sharing_a_word);
                    }
                }
                num_of_shared_words[view_sharing_a_word] += 1; //pKFi->mnLoopWords++;
            }
        }
        
        return views_sharing_words;
    }
    
    
    
    std::vector<View*> ViewDatabase::detectLoopCandidates(View &view, double min_score)
    {
        std::vector<View *> loop_candidates;
        
        std::map<View*, int> num_of_shared_words;
        std::list<View *> views_sharing_words = findViewsSharingWords(view, num_of_shared_words);
        
        if (views_sharing_words.empty())
        {
            return loop_candidates;
        }
        
        // Only compare against those views that share enough words
        
        
        int max_common_words = 0;
        for (const auto &vi: views_sharing_words)
        {
            if (num_of_shared_words[vi] > max_common_words)
            {
                max_common_words = num_of_shared_words[vi];
            }
        }
        int min_common_words = max_common_words*0.8f;
        
        
        // Compute similarity score. Retain the matches whose score is higher than minScore
        std::map<View*, int> scores;
        std::list<std::pair<double, View *>> score_and_view_pairs;
        
        const auto &voc = ORBVocabulary::instance().vocabulary();
        for (const auto &vi: views_sharing_words)
        {
            if (num_of_shared_words[vi] > min_common_words)
            {
                double s = voc.score(view.frame().bow(), vi->frame().bow());
                
                scores[vi] = s;
                if (s >= min_score)
                {
                    score_and_view_pairs.push_back( std::make_pair(s, vi) );
                }
            }
        }
        
        if ( score_and_view_pairs.empty() )
        {
            return loop_candidates;
        }
        
        
        // ---------------------------------------------------------
        // --  Lets now accumulate score by covisibility
        // ---------------------------------------------------------
        
        std::list< std::pair<double, View*> > acc_score_and_view_pairs;
        double best_acc_score = min_score;
        
        //  for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
        for (const auto &pair: score_and_view_pairs)
        {
            const auto &vi = pair.second; //KeyFrame* pKFi = it->second;
            
            auto covisibility = vi->getBestCovisibilityViews(10); //vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);
            double best_score = pair.first;
            double acc_score = pair.first;
            
            View *best_view = vi;
            //      for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
            for(View *co_view: covisibility) //KeyFrame* pKF2 = *vit;
            {
                double co_view_score = scores[co_view];
                //if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
                // check!!!
                //if( cached_loops[co_view]==vi  &&  num_of_shared_words[co_view] > min_common_words)
                //if( num_of_shared_words.count(co_view)  &&  num_of_shared_words[co_view] > min_common_words)
                if( num_of_shared_words[co_view] > min_common_words)
                {
                    acc_score += co_view_score;      // accScore+=pKF2->mLoopScore;
                    if( co_view_score > best_score ) // if(pKF2->mLoopScore>bestScore)
                    {
                        best_view = co_view;        // pBestKF=pKF2;
                        best_score = co_view_score; // bestScore = pKF2->mLoopScore;
                    }
                }
            }
            
            acc_score_and_view_pairs.push_back(std::make_pair(acc_score,best_view));
            if(acc_score > best_acc_score)
            {
                best_acc_score = acc_score;
            }
        }
        
        
        // --------------------------------------------------------------------
        // Return all those keyframes with a score higher than 0.75*bestScore
        // --------------------------------------------------------------------
        double min_score_to_retain = 0.75f*best_acc_score;
        
        std::set<View*> already_added_views;
        loop_candidates.reserve(acc_score_and_view_pairs.size());
        
        for (const auto &pair : acc_score_and_view_pairs)
        {
            auto &score = pair.first;
            if (score > min_score_to_retain)
            {
                const auto &vi = pair.second;
                if (!already_added_views.count(vi))
                {
                    loop_candidates.push_back(vi);
                    already_added_views.insert(vi);
                }
            }
        }
        
        return loop_candidates;
    }

    

//
//    vector<KeyFrame*> KeyViewDatabase::DetectRelocalizationCandidates(Frame *F)
//    {
//        list<KeyFrame*> lKFsSharingWords;
//
//        // Search all keyframes that share a word with current frame
//        {
//            unique_lock<mutex> lock(mMutex);
//
//            for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
//            {
//                list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];
//
//                for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
//                {
//                    KeyFrame* pKFi=*lit;
//                    if(pKFi->mnRelocQuery!=F->mnId)
//                    {
//                        pKFi->mnRelocWords=0;
//                        pKFi->mnRelocQuery=F->mnId;
//                        lKFsSharingWords.push_back(pKFi);
//                    }
//                    pKFi->mnRelocWords++;
//                }
//            }
//        }
//        if(lKFsSharingWords.empty())
//            return vector<KeyFrame*>();
//
//        // Only compare against those keyframes that share enough words
//        int maxCommonWords=0;
//        for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
//        {
//            if((*lit)->mnRelocWords>maxCommonWords)
//                maxCommonWords=(*lit)->mnRelocWords;
//        }
//
//        int minCommonWords = maxCommonWords*0.8f;
//
//        list<pair<float,KeyFrame*> > lScoreAndMatch;
//
//        int nscores=0;
//
//        // Compute similarity score.
//        for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
//        {
//            KeyFrame* pKFi = *lit;
//
//            if(pKFi->mnRelocWords>minCommonWords)
//            {
//                nscores++;
//                float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
//                pKFi->mRelocScore=si;
//                lScoreAndMatch.push_back(make_pair(si,pKFi));
//            }
//        }
//
//        if(lScoreAndMatch.empty())
//            return vector<KeyFrame*>();
//
//        list<pair<float,KeyFrame*> > lAccScoreAndMatch;
//        float bestAccScore = 0;
//
//        // Lets now accumulate score by covisibility
//        for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
//        {
//            KeyFrame* pKFi = it->second;
//            vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);
//
//            float bestScore = it->first;
//            float accScore = bestScore;
//            KeyFrame* pBestKF = pKFi;
//            for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
//            {
//                KeyFrame* pKF2 = *vit;
//                if(pKF2->mnRelocQuery!=F->mnId)
//                    continue;
//
//                accScore+=pKF2->mRelocScore;
//                if(pKF2->mRelocScore>bestScore)
//                {
//                    pBestKF=pKF2;
//                    bestScore = pKF2->mRelocScore;
//                }
//
//            }
//            lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
//            if(accScore>bestAccScore)
//                bestAccScore=accScore;
//        }
//
//        // Return all those keyframes with a score higher than 0.75*bestScore
//        float minScoreToRetain = 0.75f*bestAccScore;
//        set<KeyFrame*> spAlreadyAddedKF;
//        vector<KeyFrame*> vpRelocCandidates;
//        vpRelocCandidates.reserve(lAccScoreAndMatch.size());
//        for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
//        {
//            const float &si = it->first;
//            if(si>minScoreToRetain)
//            {
//                KeyFrame* pKFi = it->second;
//                if(!spAlreadyAddedKF.count(pKFi))
//                {
//                    vpRelocCandidates.push_back(pKFi);
//                    spAlreadyAddedKF.insert(pKFi);
//                }
//            }
//        }
//
//        return vpRelocCandidates;
//    }
    
}
