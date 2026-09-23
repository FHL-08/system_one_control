"""Batched Noul evaluation: one forward pass for all membership questions.

Replicates the question-form path of BertaBackend.evaluate_noul: each question
"Q?" is scored as softmax(entail_logit("Q? Yes."), entail_logit("Q? No."))[0],
but all questions share the premise and run as a single batch.
"""
import torch
from von.engine import VonEngine


def noul_batch(state_text, questions):
    eng = VonEngine.get_instance()
    be = eng.backend
    model, tok = be._get_model_and_tok()

    premises, hyps = [], []
    for s in questions:
        q = s.strip().rstrip("?")
        premises += [state_text, state_text]
        hyps += [f"{q}? Yes.", f"{q}? No."]

    inputs = tok(premises, hyps, padding=True, truncation=True,
                 max_length=512, return_tensors="pt").to(be.device)
    with torch.no_grad():
        ent = model(**inputs).logits[:, be._entail_idx]
    probs = torch.softmax(ent.view(-1, 2), dim=-1)[:, 0]
    return [round(max(0.0, min(1.0, float(p))), 4) for p in probs]
